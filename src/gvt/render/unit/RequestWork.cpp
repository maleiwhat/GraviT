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
// RequestWork.cpp
//

#include "gvt/render/unit/RequestWork.h"
#include "gvt/core/mpi/Work.h"
#include "gvt/core/mpi/Application.h"
#include "apps/render/MpiRenderer.h"

#include <iostream>

using namespace gvt::core::mpi;
using namespace gvt::render::unit;
using namespace apps::render;

#define DEBUG_TILE_DISTRIBUTION

WORK_CLASS(RequestWork)

void RequestWork::Serialize(size_t& size, unsigned char*& serialized) {

  size = sizeof(int);
  serialized = static_cast<unsigned char *>(malloc(size));
  *reinterpret_cast<int*>(serialized) = sourceRank;
}

Work* RequestWork::Deserialize(size_t size, unsigned char* serialized) {

  if (size != sizeof(int)) {
    std::cerr << "RequestWork deserializer ctor with size != sizeof(int)\n";
    exit(1);
  }
  RequestWork *requestWork = new RequestWork;
  requestWork->setSourceRank(*reinterpret_cast<int*>(serialized));
  return static_cast<Work*>(requestWork);
}

bool RequestWork::Action() {

  #ifdef DEBUG_TILE_DISTRIBUTION
  printf("Rank %d: received RequestWork from rank %d\n",
          Application::GetApplication()->GetRank(),
          getSourceRank());
  #endif

  MpiRenderer* app = static_cast<MpiRenderer*>(Application::GetApplication());
  TileLoadBalancer* loadBalancer = app->getTileLoadBalancer();

  TileWork tile = loadBalancer->Next();

  if (tile.isValid()) {
    tile.Send(getSourceRank());

    #ifdef DEBUG_TILE_DISTRIBUTION
    printf("Rank %d: sending tile (%d %d %d %d) to Rank %d\n",
          Application::GetApplication()->GetRank(),
          tile.getStartX(), tile.getStartY(),
          tile.getWidth(), tile.getHeight(),
          getSourceRank());
    #endif
  }
  // } else {
  //   Application::GetApplication()->QuitApplication();
  // }

  return false;
}
