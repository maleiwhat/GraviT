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
// DomainWorks.h
//
#ifndef GVT_RENDER_UNIT_DOMAIN_WORKS_H
#define GVT_RENDER_UNIT_DOMAIN_WORKS_H

#include "gvt/render/unit/Work.h"
#include "gvt/render/unit/Worker.h"

#include <cassert>
#include <memory>
#include <vector>

namespace gvt {
namespace render {
namespace actor {

class Ray;

} // namespace actor
} // namespace render
} // namespace gvt

namespace gvt {
namespace render {
namespace unit {

using namespace gvt::render::actor;

/**
 * A ray work used to send rays to other nodes.
 */
class RemoteRays : public Work {
  REGISTER_WORK(RemoteRays)

public:
  enum TransferType {
    Request,
    Grant
  };

  struct Header {
    int transfer_type;
    int sender;
    int instance;
    int num_rays;
  };

  struct Data {
    Header header;
    Ray *rays;
  };

  RemoteRays();

  RemoteRays(const Header &header);

  // TODO: avoid this copy
  RemoteRays(const Header &header, const std::vector<Ray> &rays);

  virtual bool Action(Worker *worker);

  int GetTransferType() const { return GetBufferPtr<Header>()->transfer_type; }
  int GetSender() const { return GetBufferPtr<Header>()->sender; }
  int GetInstance() const { return GetBufferPtr<Header>()->instance; }
  int GetNumRays() const { return GetBufferPtr<Header>()->num_rays; }
  const unsigned char *GetRayBuffer() const { return GetBufferPtr<unsigned char>() + sizeof(Header); }
};

/**
 * A voting work used to make consensus among all nodes.
 * This work is created and sent to other nodes inside the voter class.
 */
class Vote : public Work {
  REGISTER_WORK(Vote)

public:
  enum EnumType {
    PROPOSE,
    DO_COMMIT,
    DO_ABORT,
    VOTE_COMMIT,
    VOTE_ABORT,
    UNKNOWN,
    NUM_VOTE_TYPES
  };

  static std::string names[NUM_VOTE_TYPES];

  struct Data {
    int vote_type;
    int sender;
  };

  Vote() : Work() { Vote(UNKNOWN, -1); }

  Vote(int vote_type, int sender) : Work() {
    Allocate(sizeof(Data));
    GetBufferPtr<Data>()->vote_type = vote_type;
    GetBufferPtr<Data>()->sender = sender;
  }

  virtual bool Action(Worker *worker);

  // getters
  int GetVoteType() const { return GetBufferPtr<Data>()->vote_type; }
  int GetSender() const { return GetBufferPtr<Data>()->sender; }
};

} // namespace unit
} // namespace render
} // namespace gvt

#endif
