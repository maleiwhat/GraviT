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
// RenderWork.cpp
//

#include "gvt/core/mpi/RenderWork.h"
#include "gvt/core/mpi/Work.h"

using namespace gvt::core::mpi;

WORK_CLASS(TileWork)

void TileWork::intialize() {
  startX = 0;
  startY = 0;
  width = 0;
  height = 0;
}

void TileWork::Serialize(size_t& size, unsigned char*& serialized) {

  size = 4 * sizeof(int);
  serialized = (unsigned char *)malloc(size);

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
  return true;
}

WORK_CLASS(RayWork)
WORK_CLASS(GeometryWork)
WORK_CLASS(PixelWork)

