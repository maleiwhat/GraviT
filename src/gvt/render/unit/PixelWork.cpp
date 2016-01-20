/* =======================================================================================
   This file is released as part of GraviT - scalable, platform independent ray
   tracing
   tacc.github.io/GraviT

   Copyright 2013-2015 Texas Advanced Computing Center, The University of Texas
   at Austin
   All rights reserved.

   Licensed under the BSD 3-Clause License, (the "License"); you may not use
   this file
   except in compliance with the License.
   A copy of the License is included with this software in the file LICENSE.
   If your copy does not contain the License, you may obtain a copy of the
   License at:

       http://opensource.org/licenses/BSD-3-Clause

   Unless required by applicable law or agreed to in writing, software
   distributed under
   the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY
   KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under
   limitations under the License.

   GraviT is funded in part by the US National Science Foundation under awards
   ACI-1339863,
   ACI-1339881 and ACI-1339840
   =======================================================================================
   */

//
// PixelWork.cpp
//

#include "gvt/render/unit/PixelWork.h"
#include "gvt/core/mpi/Work.h"
#include "gvt/core/DatabaseNode.h"
#include "gvt/render/RenderContext.h"
#include "gvt/render/data/scene/ColorAccumulator.h"
#include "gvt/render/data/scene/Image.h"
#include "apps/render/MpiRenderer.h"

using namespace std;
using namespace gvt::core;
using namespace gvt::core::mpi;
using namespace gvt::render::unit;
using namespace gvt::render;
using namespace gvt::render::data::scene;
using namespace apps::render;

// #define DEBUG_PIXEL_WORK
// #define DEBUG_PIXEL_DESERIALIZE

WORK_CLASS(PixelWork)

void PixelWork::Serialize(size_t& size, unsigned char*& serialized) {

  size = 4 * sizeof(int) + width * height * 3 * sizeof(float);
  serialized = static_cast<unsigned char*>(malloc(size));

  unsigned char* buf = serialized;

  *reinterpret_cast<int*>(buf) = startX; buf += sizeof(int);
  *reinterpret_cast<int*>(buf) = startY; buf += sizeof(int);
  *reinterpret_cast<int*>(buf) = width;  buf += sizeof(int);
  *reinterpret_cast<int*>(buf) = height; buf += sizeof(int);

  DBNodeH root = RenderContext::instance()->getRootNode();
  int imageWidth = variant_toInteger(root["Film"]["width"].value());

  #ifdef DEBUG_PIXEL_WORK
  printf("Rank %d: serializing PixelWork tile(%d %d %d %d) imageWidth: %d\n",
         Application::GetApplication()->GetRank(),
         startX, startY, width, height, imageWidth);
  #endif

  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      int pixelId = (startY + y) * imageWidth + (startX + x);
      GVT_COLOR_ACCUM& color = (*framebuffer)[pixelId]; // float rgba[4]
      *reinterpret_cast<float*>(buf) = color.rgba[0]; buf += sizeof(float);
      *reinterpret_cast<float*>(buf) = color.rgba[1]; buf += sizeof(float);
      *reinterpret_cast<float*>(buf) = color.rgba[2]; buf += sizeof(float);

      #ifdef DEBUG_PIXEL_WORK
      printf("Rank %d: serializing PixelWork pixelId: %d \
              rgba(%.2f, %.2f, %.2f)\n",
             Application::GetApplication()->GetRank(),
             pixelId,
             color.rgba[0],
             color.rgba[1],
             color.rgba[2]);
      #endif
    }
  }
}

Work* PixelWork::Deserialize(size_t size, unsigned char* serialized) {

  unsigned char* buf = serialized;
  TileWork* pixelWork = new PixelWork;

  int tileX = *reinterpret_cast<int*>(buf); buf += sizeof(int);
  int tileY = *reinterpret_cast<int*>(buf); buf += sizeof(int);
  int tileW = *reinterpret_cast<int*>(buf); buf += sizeof(int);
  int tileH = *reinterpret_cast<int*>(buf); buf += sizeof(int);

  pixelWork->setTileSize(tileX, tileY, tileW, tileH);

  if (size != (4 * sizeof(int) + tileW * tileH * 3 * sizeof(float))) {
    std::cerr << "Test deserializer ctor with size != \
                  4 * sizeof(int) + width * height\n";
    exit(1);
  }

  DBNodeH root = RenderContext::instance()->getRootNode();
  int imageWidth = variant_toInteger(root["Film"]["width"].value());

  MpiRenderer* app = static_cast<MpiRenderer*>(Application::GetApplication());
  Image* image = app->getImage();

  #ifdef DEBUG_PIXEL_DESERIALIZE
  printf("Rank %d: de-serializing start\n",
         Application::GetApplication()->GetRank());
  #endif

  for (int y = 0; y < tileH; ++y) {
    for (int x = 0; x < tileW; ++x) {
      GVT_COLOR_ACCUM color;
      color.rgba[0] = *reinterpret_cast<float*>(buf); buf += sizeof(float);
      color.rgba[1] = *reinterpret_cast<float*>(buf); buf += sizeof(float);
      color.rgba[2] = *reinterpret_cast<float*>(buf); buf += sizeof(float);
      color.rgba[3] = 1.f;
      int pixelId = (tileY + y) * imageWidth + (tileX + x);

      image->Add(pixelId, color);

      #ifdef DEBUG_PIXEL_WORK
      printf("Rank %d: de-serializing PixelWork pixelId: %d \
              rgba(%.2f, %.2f, %.2f)\n",
             Application::GetApplication()->GetRank(),
             pixelId,
             color.rgba[0],
             color.rgba[1],
             color.rgba[2]);
      #endif
    }
  }
  #ifdef DEBUG_PIXEL_DESERIALIZE
  printf("Rank %d: de-serializing done\n",
         Application::GetApplication()->GetRank());
  #endif

  return static_cast<Work*>(pixelWork);
}

bool PixelWork::Action() {
  MpiRenderer* app = static_cast<MpiRenderer*>(Application::GetApplication());
  int count = app->decrementPendingPixelCount(width * height);
  if (count == 0) {
    app->getImage()->Write(false);
    app->QuitApplication();
  }
  return false;
}