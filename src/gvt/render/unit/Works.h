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
  WORK_CLASS_HEADER(RayTransferWork)

public:
  enum TransferType { Request, Grant };

  virtual ~RayTransferWork() {}
  virtual void Serialize(size_t &size, unsigned char *&serialized);
  static Work *Deserialize(size_t size, unsigned char *serialized);
  virtual bool Action();
  virtual bool deferDeletingThis();

  void setup(int transferType, int senderRank, int instanceId, gvt::render::actor::RayVector *outgoingRays);
  void setup(int transferType, int senderRank, int numRays);
  void copyIncomingRays(std::map<int, gvt::render::actor::RayVector> *destinationRayQ);
  int getNumRays() const { return numRays; }
  int getSenderRank() const { return senderRank; }
  int getInstanceId() const { return instanceId; }

private:
  int transferType;
  int senderRank;
  int instanceId;
  int numRays;
  gvt::render::actor::RayVector *outgoingRays;
  gvt::render::actor::RayVector incomingRays;
};

class VoteWork : public Work {
  WORK_CLASS_HEADER(VoteWork)

public:
  enum VoteType { NoWork, Resign, Commit, Abort };

  virtual ~VoteWork() {}
  virtual void Serialize(size_t &size, unsigned char *&serialized);
  static Work *Deserialize(size_t size, unsigned char *serialized);
  virtual bool Action();

  void setup(int voteType, int senderRank, unsigned int timeStamp);
  int getSenderRank() const { return senderRank; }
  unsigned int getTimeStamp() const { return timeStamp; }
  int getVoteType() const { return voteType; }

private:
  int voteType;
  int senderRank;
  unsigned int timeStamp;
};

class PixelGatherWork : public Work {
  WORK_CLASS_HEADER(PixelGatherWork)
public:
  virtual ~PixelGatherWork() {}
  virtual void Serialize(std::size_t &size, unsigned char *&serialized);
  static Work *Deserialize(std::size_t size, unsigned char *serialized);
  virtual bool Action();
};
}
}
}

#endif
