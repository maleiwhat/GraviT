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
// Voter.h
//
#ifndef GVT_RENDER_UNIT_VOTER_H
#define GVT_RENDER_UNIT_VOTER_H

#include <pthread.h>
#include <map>
#include "gvt/render/actor/Ray.h"
#include "gvt/render/unit/Types.h"

namespace gvt {
namespace render {
namespace unit {

class Worker;
class Communicator;

class Vote;
class RayTracer;

/**
 * Two phase commit voter.
 */
class Voter {
public:
  Voter(const MpiInfo &mpi, const RayTracer &tracer, Communicator *comm);

  bool isDone();
  void updateRayRx(int numRaysAck);
  void updateRayTx(int numRaysSent);
  // void UpdateVote(const Vote &vote);
  void updateState(int receivedVoteType, bool checkDone);
  // bool isCommAllowed() const;

  friend class DomainTracer;

  enum State {
    IDLE_COORDINATOR,
    IDLE_COHORT,
    WAIT_VOTE,
    WAIT_VOTE_RESULT,
    TERMINATE,
    NUM_STATES
  };

private:
  static std::string stateNames[NUM_STATES];

  enum Role { COORDINATOR = 0 };

  void sendVote(int voteWorkType) const;
  void reset();
  bool isTimeToVote() const;
  bool hasWork() const;
  bool allVotesReceived() const;
  bool achievedConsensus() const;
  void broadcast(int voteWorkType) const;

  Communicator *comm;
  const MpiInfo mpi;
  const RayTracer &tracer;

  int state;
  int numVotesReceived;
  int numCommitVotes;
  int numPendingRays;
};
}
}
}

#endif
