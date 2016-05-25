#include "gvt/render/unit/Voter.h"
#include "gvt/render/unit/Works.h"
#include "gvt/render/unit/MpiRenderer.h"

using namespace gvt::render::unit;

Voter::Voter(MpiRenderer *renderer, int numRanks, int myRank, std::map<int, gvt::render::actor::RayVector> *rayQ)
    : renderer(renderer), numRanks(numRanks), myRank(myRank), rayQ(rayQ), numPendingRays(0), allVotesAvailable(false),
      numVotesReceived(0), commitVoteCount(0), commitAbortAvailable(false), doCommit(false), proposeAvailable(false) {

  pthread_mutex_init(&votingLock, NULL);
  // pthread_mutex_init(&voteWorkBufferLock, NULL);
  if (myRank == COORDINATOR) {
    state = PREPARE_COORDINATOR;
  } else {
    state = PREPARE_COHORT;
  }

#ifndef NDEBUG
  stateNames.resize(NUM_STATES);
  stateNames = { "PREPARE_COORDINATOR", "PROPOSE", "PREPARE_COHORT", "VOTE", "TERMINATE" };
#endif
}

void Voter::reset() {
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

void Voter::addNumPendingRays(int n) {
  pthread_mutex_lock(&votingLock);
  numPendingRays += n;
  pthread_mutex_unlock(&votingLock);
}

void Voter::subtractNumPendingRays(int n) {
  pthread_mutex_lock(&votingLock);
#ifndef NDEBUG
  printf("rank %d numPendingRays(before) %d. sent %d rays. numPendingRays(after) %d in %s\n", myRank, numPendingRays, n,
         (numPendingRays - n), __PRETTY_FUNCTION__);
#endif
  numPendingRays -= n;
  assert(numPendingRays >= 0);
  pthread_mutex_unlock(&votingLock);
}

void Voter::bufferVoteWork(VoteWork *work) {
  pthread_mutex_lock(&voteWorkBufferLock);
  voteWorkBuffer.push_back(work); // TODO: avoid resizing
#ifndef NDEBUG
  printf("rank %d received vote request from rank %d voteType %d in %s\n", myRank, work->getSenderRank(),
         work->getVoteType(), __PRETTY_FUNCTION__);
#endif
  pthread_mutex_unlock(&voteWorkBufferLock);
}

bool Voter::hasWork() const {
  // lock not needed (ray queues are certainly not in use when FSM runs)
  int notDone = 0;
  for (auto &q : *rayQ) notDone += q.second.size();
  return !(notDone == 0 && numPendingRays == 0);
}

bool Voter::updateState() {
  pthread_mutex_lock(&votingLock);
#ifndef NDEBUG
  int oldState = state;
#endif
  bool allDone = false;

  switch (state) {

  case PREPARE_COORDINATOR: {
    if (!hasWork()) {
      broadcast(VoteWork::PROPOSE);
      state = PROPOSE;
    }
  } break;

  case PROPOSE: {
    if (allVotesAvailable) {
      if (achievedConsensus()) {
        broadcast(VoteWork::DO_COMMIT);
        state = TERMINATE;
        allDone = true;
      } else {
        broadcast(VoteWork::DO_ABORT);
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
        sendVote(VoteWork::VOTE_ABORT);
      } else {
        sendVote(VoteWork::VOTE_COMMIT);
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

  default: {
  } break;

  } // switch (state) {

#ifndef NDEBUG
  if (oldState != state)
    std::cout << "rank " << myRank << ": " << stateNames[oldState] << " -> " << stateNames[state] << "\n";
  else
    std::cout << "rank " << myRank << ": " << stateNames[state] << "\n";
#endif
  pthread_mutex_unlock(&votingLock);
  return allDone;
}

bool Voter::achievedConsensus() const { return (commitVoteCount == numRanks - 1); }

void Voter::broadcast(int voteWorkType) const {
  for (int i = 0; i < numRanks; ++i) {
    if (i != myRank) {
      VoteWork* work = new VoteWork(VoteWork::getSize());
      work->setup(voteWorkType, myRank);
      // work.Send(i);
      renderer->SendWork(work, i);
#ifndef NDEBUG
      printf("rank %d: sent vote request to rank %d voteWorkType %d\n", myRank, i, voteWorkType);
#endif
    }
  }
}

void Voter::sendVote(int voteWorkType) const {
  VoteWork* work = new VoteWork(VoteWork::getSize());
  work->setup(voteWorkType, myRank);
  // work.Send(COORDINATOR);
  renderer->SendWork(work, COORDINATOR);
}

void Voter::setProposeAvailable() {
  pthread_mutex_lock(&votingLock);
  proposeAvailable = true;
  pthread_mutex_unlock(&votingLock);
}

void Voter::voteCommit() {
  pthread_mutex_lock(&votingLock);
  ++commitVoteCount;
  ++numVotesReceived;
  if (numVotesReceived == numRanks - 1) allVotesAvailable = true;
  pthread_mutex_unlock(&votingLock);
}

void Voter::voteAbort() {
  pthread_mutex_lock(&votingLock);
  ++numVotesReceived;
  if (numVotesReceived == numRanks - 1) allVotesAvailable = true;
  pthread_mutex_unlock(&votingLock);
}

void Voter::commit() {
  commitAbortAvailable = true;
  doCommit = true;
}

void Voter::abort() { commitAbortAvailable = true; }

bool Voter::isCommunicationAllowed() const {
  return (myRank == COORDINATOR && state == PREPARE_COORDINATOR) || (myRank != COORDINATOR && state == PREPARE_COHORT);
}

