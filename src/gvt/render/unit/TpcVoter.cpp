#include "gvt/render/unit/TpcVoter.h"

#include "gvt/render/unit/RayTracer.h"
#include "gvt/render/unit/Worker.h"

#include "gvt/render/unit/DomainWorks.h"

#define DEBUG_VOTER

namespace gvt {
namespace render {
namespace unit {

using namespace gvt::render::unit;

TpcVoter::TpcVoter(int numRanks, int myRank, const RayTracer &tracer,
                   Worker *worker)
    : numRanks(numRanks),
      myRank(myRank),
      tracer(tracer),
      worker(worker),
      // rayQ(rayQ),
      numPendingRays(0),
      allVotesAvailable(false),
      numVotesReceived(0),
      commitVoteCount(0),
      commitAbortAvailable(false),
      doCommit(false),
      proposeAvailable(false) {
  pthread_mutex_init(&votingLock, NULL);
  if (myRank == COORDINATOR) {
    state = PREPARE_COORDINATOR;
  } else {
    state = PREPARE_COHORT;
  }

#ifdef DEBUG_VOTER
  stateNames.resize(NUM_STATES);
  stateNames = {"PREPARE_COORDINATOR", "PROPOSE", "PREPARE_COHORT", "VOTE",
                "TERMINATE"};
#endif
}

void TpcVoter::reset() {
  if (myRank == COORDINATOR) {
    state = PREPARE_COORDINATOR;
  } else {
    state = PREPARE_COHORT;
  }
  numPendingRays = 0;
  allVotesAvailable = false;
  numVotesReceived = 0;
  commitVoteCount = 0;
  commitAbortAvailable = false;
  doCommit = false;
  proposeAvailable = false;
}

void TpcVoter::addNumPendingRays(int n) {
  pthread_mutex_lock(&votingLock);
  numPendingRays += n;
  pthread_mutex_unlock(&votingLock);
}

void TpcVoter::subtractNumPendingRays(int n) {
  pthread_mutex_lock(&votingLock);
#ifdef DEBUG_VOTER
  printf(
      "rank %d numPendingRays(before) %d. sent %d rays. numPendingRays(after) "
      "%d in %s\n",
      myRank, numPendingRays, n, (numPendingRays - n), __PRETTY_FUNCTION__);
#endif
  numPendingRays -= n;
  assert(numPendingRays >= 0);
  pthread_mutex_unlock(&votingLock);
}

bool TpcVoter::hasWork() const {
  // lock not needed (ray queues are certainly not in use when FSM runs)
  // int notDone = 0;
  // for (auto &q : *rayQ) notDone += q.second.size();
  // return !(notDone == 0 && numPendingRays == 0);
  return !(tracer.IsDone() && numPendingRays == 0);
}

bool TpcVoter::updateState() {
  pthread_mutex_lock(&votingLock);
#ifdef DEBUG_VOTER
  int old_state = state;
#endif
  bool allDone = false;

  switch (state) {
    case PREPARE_COORDINATOR: {
      if (!hasWork()) {
        broadcast(Vote::PROPOSE);
        state = PROPOSE;
      }
    } break;

    case PROPOSE: {
      if (allVotesAvailable) {
        if (achievedConsensus()) {
          broadcast(Vote::DO_COMMIT);
          state = TERMINATE;
          allDone = true;
        } else {
          broadcast(Vote::DO_ABORT);
          state = PREPARE_COORDINATOR;
        }
        numVotesReceived = 0;
        commitVoteCount = 0;
        allVotesAvailable = false;
      }
    } break;

    case PREPARE_COHORT: {
      if (proposeAvailable) {
        state = VOTE;
        proposeAvailable = false;
        if (hasWork()) {
          sendVote(Vote::VOTE_ABORT);
        } else {
          sendVote(Vote::VOTE_COMMIT);
        }
      }
    } break;

    case VOTE: {
      if (commitAbortAvailable) {
        if (doCommit) {
          state = TERMINATE;
          allDone = true;
        } else {
          state = PREPARE_COHORT;
          commitAbortAvailable = false;
        }
      }
    } break;

    case TERMINATE: {
      reset();
    } break;

    default: { } break; }  // switch (state) {

#ifdef DEBUG_VOTER
  if (old_state != state)
    std::cout << "rank " << myRank << ": " << stateNames[old_state] << " -> "
              << stateNames[state] << "\n";
// else
//   std::cout << "rank " << myRank << ": " << stateNames[state] << "\n";
#endif
  pthread_mutex_unlock(&votingLock);
  return allDone;
}

bool TpcVoter::achievedConsensus() const {
  return (commitVoteCount == numRanks - 1);
}

void TpcVoter::broadcast(int voteWorkType) const {
  //   for (int i = 0; i < numRanks; ++i) {
  //     if (i != myRank) {
  //       // Vote *work = new Vote(Vote::getSize());
  //       // work->setup(voteWorkType, myRank);
  //       Vote *work = new Vote(voteWorkType, myRank);
  //       // work.Send(i);
  //       // worker->SendWork(work, i);
  //       work->Send(i, worker);
  // #ifndef NDEBUG
  //       printf("rank %d: sent vote request to rank %d voteWorkType %d\n",
  //       myRank,
  //              i, voteWorkType);
  // #endif
  //     }
  //   }
  Vote *work = new Vote(voteWorkType, myRank);
  work->SendAllOther(worker);
}

void TpcVoter::sendVote(int voteWorkType) const {
  // Vote *work = new Vote(Vote::getSize());
  // work->setup(voteWorkType, myRank);
  // // work.Send(COORDINATOR);
  // // worker->SendWork(work, COORDINATOR);
  // worker->Send(work);
  Vote *work = new Vote(voteWorkType, myRank);
  work->Send(COORDINATOR, worker);
}

void TpcVoter::setProposeAvailable() {
  pthread_mutex_lock(&votingLock);
  proposeAvailable = true;
  pthread_mutex_unlock(&votingLock);
}

void TpcVoter::voteCommit() {
  pthread_mutex_lock(&votingLock);
  ++commitVoteCount;
  ++numVotesReceived;
  if (numVotesReceived == numRanks - 1) allVotesAvailable = true;
  pthread_mutex_unlock(&votingLock);
}

void TpcVoter::voteAbort() {
  pthread_mutex_lock(&votingLock);
  ++numVotesReceived;
  if (numVotesReceived == numRanks - 1) allVotesAvailable = true;
  pthread_mutex_unlock(&votingLock);
}

void TpcVoter::commit() {
  commitAbortAvailable = true;
  doCommit = true;
}

void TpcVoter::abort() { commitAbortAvailable = true; }

bool TpcVoter::isCommunicationAllowed() const {
  return (myRank == COORDINATOR && state == PREPARE_COORDINATOR) ||
         (myRank != COORDINATOR && state == PREPARE_COHORT);
}

}  // namespace unit
}  // namespace render
}  // namespace gvt

