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
// TileWork.cpp
//

#include "gvt/render/unit/TileWork.h"
#include "gvt/render/unit/RequestWork.h"

#include "gvt/core/mpi/Application.h"
#include "gvt/core/mpi/Work.h"
#include "gvt/core/math/Vector.h"
#include "apps/render/MpiRenderer.h"
#include "gvt/render/RenderContext.h"

#include <iostream>

using namespace gvt::core::mpi;
using namespace gvt::core::math;
using namespace gvt::render::unit;
using namespace gvt::render;
using namespace apps::render;

#define DEBUG_TILE_WORK

WORK_CLASS(TileWork)

void TileWork::Serialize(size_t& size, unsigned char*& serialized) {

  size = 4 * sizeof(int);
  serialized = static_cast<unsigned char*>(malloc(size));

  unsigned char* buf = serialized;

  *reinterpret_cast<int*>(buf) = startX; buf += sizeof(int);
  *reinterpret_cast<int*>(buf) = startY; buf += sizeof(int);
  *reinterpret_cast<int*>(buf) = width;  buf += sizeof(int);
  *reinterpret_cast<int*>(buf) = height; buf += sizeof(int);
}

Work* TileWork::Deserialize(size_t size, unsigned char* serialized) {

  if (size != (4 * sizeof(int))) {
    std::cerr << "Test deserializer ctor with size != 4 * sizeof(int)\n";
    exit(1);
  }

  unsigned char* buf = serialized;
  TileWork* tileWork = new TileWork;

  tileWork->startX = *reinterpret_cast<int*>(buf); buf += sizeof(int);
  tileWork->startY = *reinterpret_cast<int*>(buf); buf += sizeof(int);
  tileWork->width  = *reinterpret_cast<int*>(buf); buf += sizeof(int);
  tileWork->height = *reinterpret_cast<int*>(buf); buf += sizeof(int);

  return static_cast<Work*>(tileWork);
}

bool TileWork::Action() {

#ifdef DEBUG_TILE_WORK
  printf("Rank %d: start processing tile(%d, %d, %d, %d)\n",
          Application::GetApplication()->GetRank(),
          startX, startY, width, height);
#endif

  RayVector rays;
  generatePrimaryRays(rays);
  // traceRays(rays);

  RequestWork request;
  request.setSourceRank(Application::GetApplication()->GetRank());
  request.Send(apps::render::rank::Server);

  return false;
}

void TileWork::set(int x, int y, int w, int h) {
  startX = x;
  startY = y;
  width = w;
  height = h;
}

void TileWork::generatePrimaryRays(RayVector& rays) {

  MpiRenderer* app = static_cast<MpiRenderer*>(Application::GetApplication());
  RenderContext* renderContext = app->getRenderContext();
  gvt::core::DBNodeH root = renderContext->getRootNode();

  int imageW = gvt::core::variant_toInteger(root["Film"]["width"].value());
  int imageH = gvt::core::variant_toInteger(root["Film"]["height"].value());
  int tileW = width;
  int tileH = height;

  Point4f eye =
      gvt::core::variant_toPoint4f(root["Camera"]["eyePoint"].value());
  float fov = gvt::core::variant_toFloat(root["Camera"]["fov"].value());
  const gvt::core::math::AffineTransformMatrix<float>& cameraToWorld =
      app->getCamera()->getCameraToWorld();

  rays.resize(tileW * tileH);

  // Generate rays direction in camera space and transform to world space.
  int i, j;
  int localIndex, globalIndex;
  float aspectRatio = float(imageW) / float(imageH);
  float x, y;
  // these basis directions are scaled by the aspect ratio and
  // the field of view.
  Vector4f camera_vert_basis_vector =
      Vector4f(0, 1, 0, 0) * tan(fov * 0.5);

  Vector4f camera_horiz_basis_vector =
      Vector4f(1, 0, 0, 0) * tan(fov * 0.5) * aspectRatio;

  Vector4f camera_normal_basis_vector = Vector4f(0, 0, 1, 0);
  Vector4f camera_space_ray_direction;

  for (j = startY; j < startY + tileH; j++)
    for (i = startX; i < startX + tileW; i++) {
      // select a ray and load it up
      localIndex = (j - startY) * tileW + (i - startX);
      globalIndex = j * imageW + i;
      Ray &ray = rays[localIndex];
      ray.id = globalIndex;
      ray.w = 1.0; // ray weight 1 for no subsamples. mod later
      ray.origin = eye;
      ray.type = Ray::PRIMARY;
      // calculate scale factors -1.0 < x,y < 1.0
      x = 2.0 * float(i) / float(imageW - 1) - 1.0;
      y = 2.0 * float(j) / float(imageH - 1) - 1.0;
      // calculate ray direction in camera space;
      camera_space_ray_direction = camera_normal_basis_vector +
                                   x * camera_horiz_basis_vector +
                                   y * camera_vert_basis_vector;
      // transform ray to world coordinate space;
      ray.setDirection(cameraToWorld * camera_space_ray_direction.normalize());
      ray.depth = 0;
    }
}

void TileWork::traceRays(const RayVector& rays) {
  // TODO
}
