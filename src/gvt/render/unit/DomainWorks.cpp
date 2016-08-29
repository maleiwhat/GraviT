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
  assert(false);
  //   TpcVoter* voter = worker->GetVoter();
  //   int num_rays = GetNumRays();
  //   int tx_type = GetTransferType();
  //
  // #ifdef DEBUG_TX
  //   printf(
  //       "ranks %d RayHeader (type %d sender %d instance %d num_rays %d) in %s\n",
  //       worker->GetRank(), tx_type, GetSender(), GetInstance(), num_rays,
  //       __PRETTY_FUNCTION__);
  // #endif
  //   // #endif
  //
  //   bool delete_this = true;
  //
  //   if (tx_type == Request) {
  //     worker->GetTracer()->EnqueWork(this);
  //     delete_this = false;
  //   } else if (tx_type == Grant) {  // Grant
  //     voter->subtractNumPendingRays(num_rays);
  //   } else {
  //     assert(false);
  //   }
  //
  //   return delete_this;
}

std::string Vote::names[Vote::NUM_VOTE_TYPES] = { "PROPOSE",     "DO_COMMIT",  "DO_ABORT",
                                                  "VOTE_COMMIT", "VOTE_ABORT", "UNKNOWN" };

bool Vote::Action(Worker *worker) {
  assert(false);
  //   TpcVoter* voter = worker->GetVoter();
  //   int type = GetVoteType();
  //
  // #ifndef NDEBUG
  //   std::string name;
  //   if (type == PROPOSE) {
  //     name = "PROPOSE";
  //   } else if (type == DO_COMMIT) {
  //     name = "DO_COMMIT";
  //   } else if (type == DO_ABORT) {
  //     name = "DO_ABORT";
  //   } else if (type == VOTE_COMMIT) {
  //     name = "VOTE_COMMIT";
  //   } else if (type == VOTE_ABORT) {
  //     name = "VOTE_ABORT";
  //   }
  //   std::cout << "[VOTER] rank " << worker->GetRank() << " got vote_type " << name
  //             << std::endl;
  // #endif
  //
  //   if (type == PROPOSE) {
  //     voter->setProposeAvailable();
  //   } else if (type == DO_COMMIT) {
  //     voter->commit();
  //   } else if (type == DO_ABORT) {
  //     voter->abort();
  //   } else if (type == VOTE_COMMIT) {
  //     voter->voteCommit();
  //   } else if (type == VOTE_ABORT) {
  //     voter->voteAbort();
  //   } else {
  //     std::cout << "rank " << worker->GetRank() << " invalid vote type " << type
  //               << std::endl;
  //     exit(1);
  //   }
  //
  //   return true;
}

} // namespace unit
} // namespace render
} // namespace gvt

