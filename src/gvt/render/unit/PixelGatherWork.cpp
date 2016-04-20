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
#include <pthread.h>
#include <iostream>

// #define DEBUG_PIXEL_GATHER_WORK
// #define DEBUG_PIXEL_GATHER_WORK_LOCK

using namespace gvt::render::unit;
using namespace gvt::render::data::scene;

WORK_CLASS(PixelGatherWork)

void PixelGatherWork::Serialize(size_t &size, unsigned char *&serialized) {
  size = 0;
  serialized = NULL;
}

Work *PixelGatherWork::Deserialize(size_t size, unsigned char *serialized) {
  if (size != 0) {
    std::cerr << "PixelGatherWork deserializer call with size != 0 rank " << Application::GetApplication()->GetRank()
              << "\n";
    exit(1);
  }
  PixelGatherWork *work = new PixelGatherWork;
  return work;
}

bool PixelGatherWork::Action() {
  MpiRenderer *renderer = static_cast<MpiRenderer *>(Application::GetApplication());
  renderer->compositePixels();
  return false;
}

