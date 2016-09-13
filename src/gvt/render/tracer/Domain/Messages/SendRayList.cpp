/* =======================================================================================
   This file is released as part of GraviT - scalable, platform independent ray tracing
   tacc.github.io/GraviT

   Copyright 2013-2015 Texas Advanced Computing Center, The University of Texas at Austin
   All rights reserved.

   Licensed under the BSD 3-Clause License, (the "License"); you may not use this file
   except in compliance with the License.
   A copy of the License is included with this software in the file LICENSE.
   If your copy does not contain the License, you may obtain a copy of the License at:

       http://opensource.org/licenses/BSD-3-Clause

   Unless required by applicable law or agreed to in writing, software distributed under
   the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
   KIND, either express or implied.
   See the License for the specific language governing permissions and limitations under
   limitations under the License.

   GraviT is funded in part by the US National Science Foundation under awards
   ACI-1339863,
   ACI-1339881 and ACI-1339840
   =======================================================================================
   */

#include "SendRayList.h"

namespace gvt {
namespace comm {

MESSAGE_HEADER_INIT(EmptyMessage);
MESSAGE_HEADER_INIT(SendRayList);

SendRayList::SendRayList(const long _src, const long _dst,
                         gvt::render::actor::RayVector &raylist) {

  std::size_t size = sizeof(gvt::render::actor::Ray) * raylist.size() + sizeof(long) * 2;
  _buffer = make_shared_buffer<unsigned char>(size + sizeof(long));
  _size = size;

  long &s = *(long *)((unsigned char *)msg_ptr() +
                      sizeof(gvt::render::actor::Ray) * raylist.size());
  long &d = *(long *)((unsigned char *)msg_ptr() +
                      sizeof(gvt::render::actor::Ray) * raylist.size() + sizeof(long));
  s = src;
  d = dst;
}
}
}
