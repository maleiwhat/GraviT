#include <gvt/core/comm/communicator.h>
#include <gvt/core/comm/vote/vote.h>

#include <cassert>
#include <iostream>

namespace gvt {
namespace comm {
namespace vote {

const char *vote::state_names[] = { "VOTE_UNKNOWN", "PROPOSE",        "DO_COMMIT",
                                    "DO_ABORT",     "COORD_FINISHED", "VOTE_COMMIT",
                                    "VOTE_ABORT",   "VOTER_FINISED",  "COMMIT",
                                    "ABORT",        "NUM_VOTE_TYPES" };

vote::vote(std::function<bool(void)> CallBackCheck,
           std::function<void(bool)> CallBackUpdate)
    : _callback_check(CallBackCheck), _callback_update(CallBackUpdate) {}

vote::vote(const vote &other) : vote(other._callback_check, other._callback_update) {
  _ballot.coordinator_id = other._ballot.coordinator_id;
  _ballot.vote = other._ballot.vote;
}

bool vote::PorposeVoting() {

  std::shared_ptr<comm::communicator> comm = comm::communicator::singleton();
  if (comm->lastid() == 1) _callback_update(true);

  if (_ballot.vote == VOTE_UNKNOWN || _ballot.vote > VOTER_FINISED) {
    std::shared_ptr<gvt::comm::Message> msg =
        std::make_shared<gvt::comm::Message>(sizeof(Ballot));
    msg->system_tag(CONTROL_VOTE_TAG);
    Ballot &b = *msg->getMessage<Ballot>();
    b.coordinator_id = _ballot.coordinator_id = comm->id();
    b.vote = _ballot.vote = PROPOSE;
    comm->broadcast(msg);
    _count = 0;
    return true;
  }
  return false;
}

// void vote::setCallBack(std::function<bool(void)> fn) {}

void vote::processMessage(std::shared_ptr<comm::Message> msg) {
  assert(_callback_check && _callback_update);

  std::cout << "-" << std::endl;
  std::shared_ptr<gvt::comm::communicator> comm = comm::communicator::singleton();
  Ballot &b = *msg->getMessage<Ballot>();

  if (_ballot.coordinator_id == comm->id()) {
    processCoordinator(msg);
    return;
  }

  if (_ballot.coordinator_id != comm->id()) {
    processCohort(msg);
    return;
  }
}

void vote::processCoordinator(std::shared_ptr<comm::Message> msg) {
  assert(_callback_check && _callback_update);
  std::shared_ptr<gvt::comm::communicator> comm = comm::communicator::singleton();
  Ballot &b = *msg->getMessage<Ballot>();

  if (b.coordinator_id < comm->id() && b.vote == PROPOSE && _ballot.vote <= PROPOSE) {
    _ballot.coordinator_id = b.coordinator_id;
    _ballot.vote = VOTE_UNKNOWN;
    _count = 0;
    processCohort(msg);
    return;
  }

  if (b.coordinator_id == _ballot.coordinator_id && _ballot.vote == PROPOSE) {
    switch (b.vote) {
    case VOTE_COMMIT: {
      _count++;
      if (_count == (comm->lastid() - 1) && _callback_check()) {
        b.vote = DO_COMMIT;
        comm->broadcast(msg);
        _ballot.vote = VOTE_UNKNOWN;
        _callback_update(true);
      }
      return;
    }
    case VOTE_ABORT: {
      _count++;
      b.vote = _ballot.vote = DO_ABORT;
      comm->broadcast(msg);
      _callback_update(false);
      if (_count == (comm->lastid() - 1)) _ballot.vote = VOTE_UNKNOWN;
      return;
    }
    default:
      break;
    }
  }

  if (b.coordinator_id == _ballot.coordinator_id && _ballot.vote == DO_ABORT) {
    _count++;
    b.vote = _ballot.vote = DO_ABORT;
    comm->broadcast(msg);
    if (_count == (comm->lastid() - 1)) _ballot.vote = VOTE_UNKNOWN;
    return;
  }
}

void vote::processCohort(std::shared_ptr<comm::Message> msg) {
  std::shared_ptr<comm::communicator> comm = comm::communicator::singleton();
  Ballot &b = *msg->getMessage<Ballot>();

  if (b.vote == PROPOSE && _ballot.coordinator_id == -1) {
    _ballot.coordinator_id = b.coordinator_id;
    _ballot.vote = VOTE_UNKNOWN;
  }

  if (b.coordinator_id == _ballot.coordinator_id && b.vote == PROPOSE &&
      _ballot.vote < VOTER_FINISED) {
    _ballot.vote = b.vote = _callback_check() ? VOTE_COMMIT : VOTE_ABORT;
    comm->send(msg, b.coordinator_id);
    return;
  }

  if (b.coordinator_id == _ballot.coordinator_id && b.vote == DO_COMMIT &&
      _ballot.vote < VOTER_FINISED) {
    _ballot.vote = b.vote = VOTE_UNKNOWN;
    _callback_update(true);
    return;
  }

  if (b.coordinator_id == _ballot.coordinator_id && b.vote == DO_ABORT &&
      _ballot.vote < VOTER_FINISED) {
    _ballot.vote = b.vote = VOTE_UNKNOWN;
    _callback_update(false);
    return;
  }
}
}
}
}
