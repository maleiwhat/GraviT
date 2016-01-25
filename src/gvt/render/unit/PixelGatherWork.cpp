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
// PixelGatherWork.cpp
//

#include "gvt/render/unit/PixelGatherWork.h"
#include "gvt/render/unit/MpiRenderer.h"
#include "gvt/render/data/scene/Image.h"

#include <vector>
#include <future>
#include <thread>
#include <iostream>

#define DEBUG_PIXEL_GATHER_WORK

using namespace gvt::render::unit;
using namespace gvt::render::data::scene;

WORK_CLASS(PixelGatherWork)

void PixelGatherWork::Serialize(size_t& size, unsigned char*& serialized) {
  size = 0;
  serialized = NULL;;
}

Work* PixelGatherWork::Deserialize(size_t size, unsigned char* serialized) {
  if (size != 0) {
    std::cerr << "PixelGatherWork deserializer call with size != 0 rank " << Application::GetApplication()->GetRank() << "\n";
    exit(1);
  }
  PixelGatherWork* work = new PixelGatherWork;
  return work;
}

bool PixelGatherWork::Action() {

  MpiRenderer* renderer
      = static_cast<MpiRenderer*>(Application::GetApplication());
  int width = renderer->getImageWidth();
  int height = renderer->getImageHeight();
  Image* image = renderer->getImage(); 
  std::vector<GVT_COLOR_ACCUM>* framebuffer = renderer->getFramebuffer();

  bool displayRank = (renderer->GetRank() == 0);
  int numRanks = renderer->GetSize();

  size_t size = width * height;

  for (size_t i = 0; i < size; i++)
    image->Add(i, (*framebuffer)[i]);

  // if (!mpi)
  //   return;

  unsigned char* rgb = image->GetBuffer();
  int rgb_buf_size = 3 * size;

  unsigned char* bufs =
      displayRank ? new unsigned char[numRanks * rgb_buf_size] : NULL;

  MPI_Gather(rgb, rgb_buf_size, MPI_UNSIGNED_CHAR,
             bufs, rgb_buf_size, MPI_UNSIGNED_CHAR,
             0, MPI_COMM_WORLD);

  if (displayRank) {
    int nchunks = std::thread::hardware_concurrency() * 2;
    int chunk_size = size / nchunks;
    std::vector<std::pair<int, int>> chunks(nchunks);
    std::vector<std::future<void>> futures;
    for (int ii = 0; ii < nchunks - 1; ii++) {
      chunks.push_back(
          std::make_pair(ii * chunk_size, ii * chunk_size + chunk_size));
    }
    int ii = nchunks - 1;
    chunks.push_back(std::make_pair(ii * chunk_size, size));
    for (auto &limit : chunks) {
      futures.push_back(std::async(std::launch::async, [&]() {
        // // std::pair<int,int> limit = std::make_pair(0,size);
        // for (size_t i = 1; i < mpi.world_size; ++i) {
        for (size_t i = 1; i < numRanks; ++i) {
          for (int j = limit.first * 3; j < limit.second * 3; j += 3) {
            int p = i * rgb_buf_size + j;
            // assumes black background, so adding is fine (r==g==b== 0)
            rgb[j + 0] += bufs[p + 0];
            rgb[j + 1] += bufs[p + 1];
            rgb[j + 2] += bufs[p + 2];
            // printf("%d (%f %f %f)\n", p, rgb[j], rgb[j+1], rgb[j+2]);
          }
        }
      }));
    }
    for (std::future<void> &f : futures) {
      f.wait();
    }

    image->Write(false);
  }
  delete[] bufs;

  return true;
}