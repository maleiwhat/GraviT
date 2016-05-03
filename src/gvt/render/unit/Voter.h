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

#ifndef GVT_RENDER_UNIT_VOTER_H
#define GVT_RENDER_UNIT_VOTER_H

#include "gvt/render/actor/Ray.h"
#include <map>
#include <pthread.h>

namespace gvt {
namespace render {
namespace unit {

class VoteWork;

class Voter {
public:
  Voter(int numRanks, int myRank, std::map<int, gvt::render::actor::RayVector> *rayQ);

  void reset();
  bool updateState();
  void addNumPendingRays(int n);
  void subtractNumPendingRays(int n);

  void bufferVoteWork(VoteWork *work);

  void setProposeAvailable();
  void voteCommit();
  void voteAbort();
  void commit();
  void abort();
  bool isCommunicationAllowed() const;

private:
  friend class MpiRenderer;
  enum State { PREPARE_COORDINATOR = 0, PROPOSE, PREPARE_COHORT, VOTE, TERMINATE, NUM_STATES };
  enum Role { COORDINATOR = 0 };

  bool hasWork() const;
  bool achievedConsensus() const;
  void broadcast(int voteWorkType) const;
  void sendVote(int voteWorkType) const;

  const int numRanks;
  const int myRank;
  const std::map<int, gvt::render::actor::RayVector> *rayQ;

  int state;
  pthread_mutex_t votingLock;
  int numPendingRays;
  bool allVotesAvailable;
  int numVotesReceived;
  int commitVoteCount;
  bool proposeAvailable;
  bool commitAbortAvailable;
  bool doCommit;

  std::vector<VoteWork *> voteWorkBuffer;
  pthread_mutex_t voteWorkBufferLock;

  std::vector<std::string> stateNames;
};
}
}
}

#endif
