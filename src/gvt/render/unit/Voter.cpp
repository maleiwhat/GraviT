#include "gvt/render/unit/Voter.h"
#include "gvt/render/unit/Works.h"

using namespace gvt::render::unit;

Voter::Voter(int numRanks, int myRank, std::map<int, gvt::render::actor::RayVector> *rayQ)
    : numRanks(numRanks), myRank(myRank), rayQ(rayQ), state(WaitForNoWork), numPendingRays(0), validTimeStamp(0),
      votesAvailable(false), resignGrant(false), numVotesReceived(0), commitCount(0), numPendingVotes(0) {

  pthread_mutex_init(&votingLock, NULL);
  // pthread_mutex_init(&voteWorkBufferLock, NULL);
}

void Voter::addNumPendingRays(int n) {
  pthread_mutex_lock(&votingLock);
  numPendingRays += n;
  pthread_mutex_unlock(&votingLock);
}

void Voter::subtractNumPendingRays(int n) {
  pthread_mutex_lock(&votingLock);
#ifdef DEBUG_RAYTX
  printf("rank %d: Voter::subtractNumPendingRays: numPendingRays(before) %d numPendingRays(after) %d\n", myRank,
         numPendingRays, (numPendingRays - n));
#endif
  numPendingRays -= n;
  assert(numPendingRays >= 0);
  pthread_mutex_unlock(&votingLock);
}

void Voter::bufferVoteWork(VoteWork *work) {
  pthread_mutex_lock(&voteWorkBufferLock);
  voteWorkBuffer.push_back(work); // TODO: avoid resizing
#ifdef DEBUG_RAYTX
  printf("rank %d: Voter::bufferVoteWork received vote request from rank %d voteType %d timeStamp %d\n", myRank,
         work->getSenderRank(), work->getVoteType(), work->getTimeStamp());
#endif
  pthread_mutex_unlock(&voteWorkBufferLock);
}

void Voter::voteForResign(int senderRank, unsigned int timeStamp) {
  int vote = (state == WaitForResign || state == Resigned) ? VoteWork::Commit : VoteWork::Abort;
  VoteWork grant;
  grant.setup(vote, myRank, timeStamp);
  grant.Send(senderRank);
}

void Voter::voteForNoWork(int senderRank, unsigned int timeStamp) {
  int vote = (state != WaitForNoWork) ? VoteWork::Commit : VoteWork::Abort;
  VoteWork grant;
  grant.setup(vote, myRank, timeStamp);
  grant.Send(senderRank);
}
void Voter::applyVoteResult(int voteType, unsigned int timeStamp) {
  pthread_mutex_lock(&votingLock);
  --numPendingVotes;
  assert(numPendingVotes >= 0);
  if (timeStamp == validTimeStamp) {
    ++numVotesReceived;
    commitCount += (voteType == VoteWork::Commit);
    assert(numVotesReceived < numRanks);
    if (numVotesReceived == numRanks - 1) {
      votesAvailable = true;
#ifdef DEBUG_RAYTX
      printf("rank %d: Voter::applyVoteResult: votesAvailable %d, numPendingVotes %d, numVotesReceived %d, commitCount "
             "%d voteType %d\n",
             myRank, votesAvailable, numPendingVotes, numVotesReceived, commitCount, voteType);
#endif
    }
  }
  pthread_mutex_unlock(&votingLock);
}

bool Voter::updateState() {
  pthread_mutex_lock(&votingLock);

  bool hasWork = !(rayQ->empty() && numPendingRays == 0);
  bool allDone = false;

  switch (state) {
  case WaitForNoWork: { // has work
    if (!hasWork) {
      requestForVotes(VoteWork::NoWork, validTimeStamp);
      state = WaitForVotes;
#ifdef DEBUG_RAYTX
      printf("rank %d: WaitForNoWork -> WaitForVotes\n", myRank);
#endif
    }
  } break;
  case WaitForVotes: { // pending votes
    if (hasWork) {
      ++validTimeStamp;
      numVotesReceived = 0;
      commitCount = 0;
      votesAvailable = false;
      state = WaitForNoWork;
#ifdef DEBUG_RAYTX
      printf("rank %d: WaitForVotes -> WaitForNoWork\n", myRank);
#endif
    } else if (votesAvailable) {
      bool commit = checkVotes();

      numVotesReceived = 0;
      commitCount = 0;
      votesAvailable = false;

      if (commit) {
        requestForVotes(VoteWork::Resign, validTimeStamp);
        state = WaitForResign;
#ifdef DEBUG_RAYTX
        printf("rank %d: updateState(): WaitForVotes -> WaitForResign\n", myRank);
#endif
      } else {
#ifdef DEBUG_RAYTX
        printf("rank %d: updateState(): abort message received. retrying requestForVotes.\n", myRank);
#endif
        // TODO: hpark adjust request interval based on workloads?
        requestForVotes(VoteWork::NoWork, validTimeStamp);
      }
    }
  } break;
  case WaitForResign: {
    if (hasWork) {   // TODO: hpark possible? can't just think of this case
      assert(false); // TODO: hpark let's disable this for now
      ++validTimeStamp;
      numVotesReceived = 0;
      commitCount = 0;
      votesAvailable = false;
      state = WaitForNoWork;

    } else if (votesAvailable) {
      bool commit = checkVotes();

      numVotesReceived = 0;
      commitCount = 0;
      votesAvailable = false;

      if (commit) {
        allDone = true;
        state = Resigned;
#ifdef DEBUG_RAYTX
        printf("rank %d: WaitForResign allDone=true\n", myRank);
#endif
      } else {
        requestForVotes(VoteWork::Resign, validTimeStamp);
      }
    }
  } break;
  case Resigned: {
  } break;
  default: { state = WaitForNoWork; } break;
  }
  // vote();
  pthread_mutex_unlock(&votingLock);
  return allDone;
}

void Voter::vote() {
  pthread_mutex_lock(&voteWorkBufferLock);
  for (size_t i = 0; i < voteWorkBuffer.size(); ++i) {
    VoteWork *request = voteWorkBuffer[i];
#ifdef DEBUG_RAYTX
    printf("rank %d: processing vote request type=%d timeStamp=%d\n", myRank, request->getVoteType(),
           request->getTimeStamp());
#endif
    int vote;
    int type = request->getVoteType();
    if (type == VoteWork::NoWork) {
      vote = (state != WaitForNoWork) ? VoteWork::Commit : VoteWork::Abort;
      // } else if (type == VoteWork::Resign) {
      //   vote = (state == WaitForResign) ?  VoteWork::Commit : VoteWork::Abort;
    } else {
      assert(false);
    }
    VoteWork grant;
    grant.setup(vote, myRank, request->getTimeStamp());
    grant.Send(request->getSenderRank());
#ifdef DEBUG_RAYTX
    printf("rank %d: sent vote voteType %d timeStamp %d to rank %d in response to vote Type %d (state %d "
           "numPendingVotes %d)\n",
           myRank, vote, request->getTimeStamp(), request->getSenderRank(), type, state, numPendingVotes);
#endif
    delete request;
  }
  voteWorkBuffer.clear(); // TODO: avoid this
  pthread_mutex_unlock(&voteWorkBufferLock);
}

bool Voter::checkVotes() { return (commitCount == numRanks - 1); }

void Voter::requestForVotes(int voteType, unsigned int timeStamp) {
  numPendingVotes += (numRanks - 1);
  for (int i = 0; i < numRanks; ++i) {
    if (i != myRank) {
      VoteWork work;
      work.setup(voteType, myRank, timeStamp);
      work.Send(i);
#ifdef DEBUG_RAYTX
      printf("rank %d: sent vote request to rank %d voteType %d timeStamp %d\n", myRank, i, voteType, timeStamp);
#endif
    }
  }
}
