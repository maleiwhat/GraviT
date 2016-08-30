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

   GraviT is funded in part by the US National Science Foundation under awards ACI-1339863,
   ACI-1339881 and ACI-1339840
   ======================================================================================= */
//
// DomainWorks.cpp
//
#include "gvt/render/unit/DomainWorks.h"
#include "gvt/render/unit/Work.h"

#include "gvt/render/unit/TpcVoter.h"

#include "gvt/render/actor/Ray.h"

namespace gvt {
namespace render {
namespace unit {

using namespace gvt::render::actor;

STATIC_WORK_TAG(RemoteRays)
STATIC_WORK_TAG(Vote)

RemoteRays::RemoteRays() : Work() {
  Header h = { -1 /* transfer_type */, -1 /* sender */, -1 /* instance */, 0 /* num_rays */ };
  Allocate(sizeof(Header));
  memcpy(GetBufferPtr<void>(), &h, sizeof(Header));
}

RemoteRays::RemoteRays(const Header &header) : Work() {
  Allocate(sizeof(Header));
  memcpy(GetBufferPtr<void>(), &header, sizeof(Header));
}

// TODO: avoid this copy
RemoteRays::RemoteRays(const Header &header, const std::vector<Ray> &rays) : Work() {
  std::size_t header_size = sizeof(Header);
  std::size_t rays_size = header.num_rays * sizeof(Ray);
  Allocate(header_size + rays_size);

  unsigned char *buf = GetBufferPtr<unsigned char>();
  memcpy(buf, &header, header_size);
  memcpy(buf + header_size, &rays[0], rays_size);
}

bool RemoteRays::Action(Worker *worker) {
  worker->GetTracer()->EnqueWork(this);
  return false;
}

std::string Vote::names[Vote::NUM_VOTE_TYPES] = { "PROPOSE",     "DO_COMMIT",  "DO_ABORT",
                                                  "VOTE_COMMIT", "VOTE_ABORT", "UNKNOWN" };

bool Vote::Action(Worker *worker) {
  worker->GetTracer()->EnqueWork(this);
  return false;
}

} // namespace unit
} // namespace render
} // namespace gvt

