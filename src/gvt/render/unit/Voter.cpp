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
// Voter.cpp
//
#include "gvt/render/unit/Voter.h"

#include "gvt/render/unit/Communicator.h"
#include "gvt/render/unit/RayTracer.h"

#include "gvt/render/unit/DomainWorks.h"

#include <string>
#include <iostream>

// #define DEBUG_VOTER

namespace gvt {
namespace render {
namespace unit {

void DebugState(int rank, const std::string &old_state, const std::string &new_state) {
#if 0
  std::cout << "Rank " << rank << " " << old_state << "->" << new_state << std::endl;
#endif
}

std::string Voter::stateNames[Voter::NUM_STATES] = { "IDLE_COORDINATOR", "IDLE_COHORT", "WAIT_VOTE",
                                                     "WAIT_VOTE_RESULT", "TERMINATE" };

Voter::Voter(const MpiInfo &mpi, const RayTracer &tracer, Communicator *comm) : mpi(mpi), tracer(tracer), comm(comm) {
  reset();
}

void Voter::reset() {
  state = mpi.rank == COORDINATOR ? IDLE_COORDINATOR : IDLE_COHORT;
#ifdef DEBUG_VOTER
  std::cout << "rank " << mpi.rank << " voter " << stateNames[TERMINATE] << " -> " << stateNames[state] << std::endl;
#endif
  numPendingRays = 0;
  numVotesReceived = 0;
  numCommitVotes = 0;
}

// bool Voter::isDone() {
//   if (mpi.rank == COORDINATOR) {
//     updateState(Vote::UNKNOWN /* unused */, isTimeToVote() /*checkDone */);
//     if (isTimeToVote()) {
//       assert(state == IDLE);
//       state = WAIT_VOTE;
//       DebugState(mpi.rank, "IDLE", "WAIT_VOTE");
//       broadcast(Vote::PROPOSE);
//     }
//   }
//   bool done = (state == TERMINATE);
//   if (done) reset();
//   return done;
// }

bool Voter::isDone() {
  if (mpi.rank == COORDINATOR) {
    updateState(Vote::UNKNOWN /* unused */, true /* checkDone */);
  }
  bool done = (state == TERMINATE);
  if (done) {
    assert(!hasWork());
    reset();
  }
  return done;
}

void Voter::updateRayRx(int numRaysAck) {
  // #ifdef DEBUG_VOTER
  //   printf("rank %d numPendingRays(before) %d. sent %d rays. numPendingRays(after) "
  //          "%d in %s\n",
  //          mpi.rank, numPendingRays, n, (numPendingRays - n), __PRETTY_FUNCTION__);
  // #endif
  numPendingRays -= numRaysAck;
  assert(numPendingRays >= 0);
}

void Voter::updateRayTx(int numRaysSent) { numPendingRays += numRaysSent; }

// void Voter::UpdateVote(const Vote& vote) {
//   int type = vote->GetVoteType();
//
// #ifdef DEBUG_VOTER
//   std::string name;
//   if (type == Vote::PROPOSE) {
//     name = "PROPOSE";
//   } else if (type == Vote::DO_COMMIT) {
//     name = "DO_COMMIT";
//   } else if (type == Vote::DO_ABORT) {
//     name = "DO_ABORT";
//   } else if (type == Vote::VOTE_COMMIT) {
//     name = "VOTE_COMMIT";
//   } else if (type == Vote::VOTE_ABORT) {
//     name = "VOTE_ABORT";
//   }
//   std::cout << "[VOTER] rank " << mpi.rank << " got vote_type " << name << std::endl;
// #endif
//
//   if (type == Vote::PROPOSE) { // cohorts
// #ifdef DEBUG_VOTER
//     if (!(mpi.rank != COORDINATOR && state == IDLE)) {
//       printf("assertion_error rank %d state %d\n", mpi.rank, state);
//     }
// #endif
//     assert(mpi.rank != COORDINATOR && state == IDLE);
//     state = WAIT_VOTE_RESULT;
//     if (hasWork()) {
//       sendVote(Vote::VOTE_ABORT);
//     } else {
//       sendVote(Vote::VOTE_COMMIT);
//     }
//     DebugState(mpi.rank, "IDLE", "WAIT_VOTE_RESULT");
//   } else if (type == Vote::DO_COMMIT) { // cohorts
//     assert(mpi.rank != COORDINATOR && state == WAIT_VOTE_RESULT);
//     state = TERMINATE;
//     DebugState(mpi.rank, "WAIT_VOTE_RESULT", "TERMINATE");
//   } else if (type == Vote::DO_ABORT) { // cohorts
//     assert(mpi.rank != COORDINATOR && state == WAIT_VOTE_RESULT);
//     state = IDLE;
//     DebugState(mpi.rank, "WAIT_VOTE_RESULT", "IDLE");
//   } else if (type == Vote::VOTE_COMMIT || type == Vote::VOTE_ABORT) { // coordinator
//     assert(mpi.rank == COORDINATOR && state == WAIT_VOTE);
//     numCommitVotes += (type == Vote::VOTE_COMMIT);
//     ++numVotesReceived;
//     if (allVotesReceived()) {
//       if (achievedConsensus()) {
//         state = TERMINATE;
//         DebugState(mpi.rank, "WAIT_VOTE", "TERMINATE");
//         broadcast(Vote::DO_COMMIT);
//       } else {
//         state = IDLE;
//         DebugState(mpi.rank, "WAIT_VOTE", "IDLE");
//         broadcast(Vote::DO_ABORT);
//       }
//       numVotesReceived = 0;
//       numCommitVotes = 0;
//     }
//   } else { // unknown vote type
//     std::cout << "rank " << mpi.rank << " invalid vote type " << type << std::endl;
//     exit(1);
//   }
// }

void Voter::updateState(int receivedVoteType, bool checkDone) {
  assert((!checkDone && receivedVoteType != Vote::UNKNOWN) ||
         (checkDone && receivedVoteType == Vote::UNKNOWN && mpi.rank == COORDINATOR));
#ifdef DEBUG_VOTER
  int old_state = state;
#endif
  auto &type = receivedVoteType;

  switch (state) {

  case IDLE_COHORT: {
    assert(mpi.rank != COORDINATOR);
    if (type == Vote::PROPOSE) { // cohorts
      state = WAIT_VOTE_RESULT;
      if (hasWork()) {
        sendVote(Vote::VOTE_ABORT);
      } else {
        sendVote(Vote::VOTE_COMMIT);
      }
    }
  } break;

  case WAIT_VOTE_RESULT: { // cohorts
    assert(mpi.rank != COORDINATOR && (type == Vote::DO_COMMIT || type == Vote::DO_ABORT));
    if (type == Vote::DO_COMMIT) { // cohorts
      assert(!hasWork());
      state = TERMINATE;
    } else if (type == Vote::DO_ABORT) {
      state = IDLE_COHORT;
    }
  } break;

  case IDLE_COORDINATOR: {
    assert(mpi.rank == COORDINATOR);
    if (checkDone && isTimeToVote()) {
      state = WAIT_VOTE;
      broadcast(Vote::PROPOSE);
    }
  } break;

  case WAIT_VOTE: {
    assert(mpi.rank == COORDINATOR);
    if (!checkDone) {
      if (type == Vote::VOTE_COMMIT) {
        ++numCommitVotes;
      }
      ++numVotesReceived;
      if (allVotesReceived()) {
#ifdef DEBUG_VOTER
        printf("rank 0 receved a vote(%s) num_votes_received(%d) num_commit_votes(%d)\n", Vote::names[type].c_str(),
               numVotesReceived, numCommitVotes);
#endif
        if (achievedConsensus()) {
          assert(!hasWork());
          state = TERMINATE;
          broadcast(Vote::DO_COMMIT);
        } else {
          state = IDLE_COORDINATOR;
          broadcast(Vote::DO_ABORT);
        }
        numVotesReceived = 0;
        numCommitVotes = 0;
      }
    }
  } break;

  case TERMINATE: // coordinator and cohorts
    break;

  default:
    break;
  }

#ifdef DEBUG_VOTER
  if (state != old_state) {
    std::cout << "rank " << mpi.rank << " voter " << stateNames[old_state] << " -> " << stateNames[state] << std::endl;
  }
#endif
}

// bool Voter::isCommAllowed() const { return (state == IDLE_COHORT || state == IDLE_COORDINATOR); }

inline bool Voter::hasWork() const {
#ifdef DEBUG_VOTER
  printf("rank %d voter states, rayQ_empty(%d) num_pending_rays(%d)\n", mpi.rank, tracer.IsRayQueueEmpty(),
         numPendingRays);
#endif
  return !(tracer.IsRayQueueEmpty() && numPendingRays == 0);
}

inline void Voter::sendVote(int voteWorkType) const {
#ifdef DEBUG_VOTER
  printf("rank %d sending vote %s to coordi\n", mpi.rank, Vote::names[voteWorkType].c_str());
#endif
  Vote *work = new Vote(voteWorkType, mpi.rank);
  work->Send(COORDINATOR, comm);
}

inline bool Voter::allVotesReceived() const { return (numVotesReceived == mpi.size - 1); }
inline bool Voter::achievedConsensus() const { return !hasWork() && (numCommitVotes == mpi.size - 1); }

inline void Voter::broadcast(int voteWorkType) const {
#ifdef DEBUG_VOTER
  printf("rank %d sending vote %s to all\n", mpi.rank, Vote::names[voteWorkType].c_str());
#endif
  Vote *work = new Vote(voteWorkType, mpi.rank);
  work->SendAllOther(comm);
}

inline bool Voter::isTimeToVote() const { return !hasWork(); }

} // namespace unit
} // namespace render
} // namespace gvt
