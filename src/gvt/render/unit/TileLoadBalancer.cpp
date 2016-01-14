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

#include "gvt/render/unit/TileLoadBalancer.h"

#include <algorithm>

using namespace gvt::render::unit;
using namespace std;

TileLoadBalancer::TileLoadBalancer(int x, int y,
                                   int width, int height,
                                   int granularity) :
  x(x), y(y), width(width), height(height), granularity(granularity) {

  if (granularity < 1)
    granularity = 1;

  size_t step = max(1, width * height);
  int stepw, steph;

  // stepw = width/sqrt(granularity);
  // steph = height*float(height)/float(width)/sqrt(granularity);
  // stepw = steph = width*height/granularity;
  stepw = width / granularity;
  steph = height / granularity;

  for(int ty = y ; ty < y + height; ty += steph) {
    for(int tx = x; tx < x + width; tx += stepw) {
      int twidth = min(stepw, (x + width) - tx);
      int theight = min(steph, (y + height) - ty);
      TileWork tile;
      tile.set(tx, ty, twidth, theight);
      tiles.push(tile);
      // printf("load balancer adding tile %d %d\n", tile.x, tile.y);
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
