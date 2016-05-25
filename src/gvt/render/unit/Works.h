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
// Works.h
//

#ifndef GVT_RENDER_UNIT_WORKS_H
#define GVT_RENDER_UNIT_WORKS_H

#include "gvt/core/mpi/Work.h"
#include "gvt/core/mpi/Application.h"
#include "gvt/render/actor/Ray.h"

using namespace std;
using namespace gvt::core::mpi;

namespace gvt {
namespace render {
namespace unit {

class MpiRenderer;

class RayTransferWork : public Work {
  WORK_CLASS(RayTransferWork, false)

public:
  enum TransferType { Request, Grant };

  struct RayInfo {
    int transferType;
    int senderRank;
    int instanceId;
    int numRays;
  };

  virtual ~RayTransferWork() {}
  // virtual void Serialize(size_t &size, unsigned char *&serialized);
  // static Work *Deserialize(size_t size, unsigned char *serialized);
  virtual bool Action();
  virtual bool deferDeletingThis();

  void setup(int transferType, int senderRank, int instanceId, const gvt::render::actor::RayVector &outgoingRays);
  void setup(int transferType, int senderRank, int numRays);
  void copyIncomingRays(std::map<int, gvt::render::actor::RayVector> *destinationRayQ);
  RayInfo getRayInfo() const {
    RayInfo info;
    memcpy(&info, contents->get(), sizeof(RayInfo));
    return info;
  }
  int getTransferType() const { return getRayInfo().transferType; }
  int getSenderRank() const { return getRayInfo().senderRank; }
  int getInstanceId() const { return getRayInfo().instanceId; }
  int getNumRays() const { return getRayInfo().numRays; }

  static std::size_t getSize(std::size_t raySize) { return sizeof(RayInfo) + raySize; }

private:
  unsigned char *getRays() { return contents->get() + sizeof(RayInfo); }

  // RayInfo rayInfo;
  gvt::render::actor::RayVector *outgoingRays;
  gvt::render::actor::RayVector incomingRays;
};

class VoteWork : public Work {
  WORK_CLASS(VoteWork, false)

public:
  enum Type { PROPOSE, DO_COMMIT, DO_ABORT, VOTE_COMMIT, VOTE_ABORT };

  struct Info {
    int voteType;
    int senderRank;
  };

  virtual ~VoteWork() {}
  // virtual void Serialize(size_t &size, unsigned char *&serialized);
  // static Work *Deserialize(size_t size, unsigned char *serialized);
  virtual bool Action();

  Info getInfo() const {
    Info info;
    memcpy(&info, contents->get(), sizeof(Info));
    return info;
  }

  void setup(int voteType, int senderRank);
  int getSenderRank() const { return getInfo().senderRank; }
  int getVoteType() const { return getInfo().voteType; }

  static std::size_t getSize() { return sizeof(VoteWork::Info); }
// private:
//   int voteType;
//   int senderRank;
};

class PixelGatherWork : public Work {
  WORK_CLASS(PixelGatherWork, true)
public:
  virtual ~PixelGatherWork() {}
  // virtual void Serialize(std::size_t &size, unsigned char *&serialized);
  // static Work *Deserialize(std::size_t size, unsigned char *serialized);
  virtual bool Action(MPI_Comm comm);
};

class TimeGatherWork : public Work {
  WORK_CLASS(TimeGatherWork, true)
public:
  virtual ~TimeGatherWork() {}
  // virtual void Serialize(std::size_t &size, unsigned char *&serialized);
  // static Work *Deserialize(std::size_t size, unsigned char *serialized);
  virtual bool Action(MPI_Comm comm);
};
}
}
}

#endif
