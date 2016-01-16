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
// TileLoadBalancer.cpp
//

#include "gvt/render/unit/TileLoadBalancer.h"
#include <algorithm>

#define DEBUG_LOAD_BALANCER

#ifdef DEBUG_LOAD_BALANCER
#include <iostream>
#include "gvt/core/mpi/Application.h"
using namespace gvt::core::mpi;
#endif

using namespace gvt::render::unit;
using namespace std;

TileLoadBalancer::TileLoadBalancer(int width, int height, int granularity) :
  width(width), height(height), granularity(granularity) {

  int stepw = width / granularity;
  int steph = height / granularity;

  for(int ty = 0 ; ty < height; ty += steph) {
    for(int tx = 0; tx < width; tx += stepw) {
      int twidth = min(stepw, width - tx);
      int theight = min(steph, height - ty);
      TileWork tile;
      tile.setTileSize(tx, ty, twidth, theight);
      tiles.push(tile);

      #ifdef DEBUG_LOAD_BALANCER
      printf("Rank %d: load balancer ctor adding tile (%d %d %d %d)\n",
              Application::GetApplication()->GetRank(),
              tx, ty, twidth, theight);
      #endif
    }
  }
  // printf("TileLoadBalancer : stepwh: %d %d tiles: %d\n", stepw, steph, tiles.size());
}

TileWork TileLoadBalancer::Next() {
  TileWork tile;
  if (!tiles.empty()) {
    tile = tiles.top();
    tiles.pop();
  }
  return tile;
}
