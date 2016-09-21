#include "acomm.h"
#include <cassert>
#include <memory>
#include <mpi.h>

#include <iostream>
namespace gvt {
namespace comm {
acomm::acomm() {}

void acomm::init(int argc, char *argv[]) {
  assert(!communicator::_instance);
  communicator::_instance = std::make_shared<acomm>();
  communicator::init(argc, argv);
}

void acomm::run() {
  while (!_terminate) {
    {

      if (!_outbox.empty()) {
        std::lock_guard<std::mutex> l(moutbox);
        // if (_outbox.empty()) continue;
        std::shared_ptr<Message> msg = _outbox.front();
        _outbox.erase(_outbox.begin());
        MPI::COMM_WORLD.Send(msg->getMessage<void>(), msg->buffer_size(), MPI::BYTE,
                             msg->dst(), CONTROL_SYSTEM_TAG);
      }

      MPI::Status status;
      if (MPI::COMM_WORLD.Iprobe(MPI::ANY_SOURCE, CONTROL_SYSTEM_TAG, status)) {
        // std::cout << "Got user msg" << std::endl;
        auto sender = status.Get_source();
        auto n_bytes = status.Get_count(MPI::BYTE);
        const auto data_size = n_bytes - sizeof(Message::header);
        std::shared_ptr<Message> msg = std::make_shared<Message>(data_size);
        MPI::COMM_WORLD.Recv(msg->getMessage<void>(), n_bytes, MPI::BYTE, sender,
                             CONTROL_SYSTEM_TAG);
        msg->size(data_size);
        std::lock_guard<std::mutex> l(minbox);
        if (msg->system_tag() == CONTROL_USER_TAG) _inbox.push_back(msg);
        if (msg->system_tag() == CONTROL_VOTE_TAG) voting->processMessage(msg);
      }
    }
  }
}

void acomm::send(std::shared_ptr<comm::Message> msg, std::size_t to) {
  std::cout << " ." << std::endl;
  assert(msg->tag() >= 0 && msg->tag() < registry_names.size());
  const std::string classname = registry_names[msg->tag()];
  assert(registry_ids.find(classname) != registry_ids.end());
  msg->src(id());
  msg->dst(to);
  std::lock_guard<std::mutex> l(moutbox);
  _outbox.push_back(msg);
};
void acomm::broadcast(std::shared_ptr<comm::Message> msg) {
  assert(msg->tag() >= 0 && msg->tag() < registry_names.size());
  const std::string classname = registry_names[msg->tag()];
  assert(registry_ids.find(classname) != registry_ids.end());
  msg->src(id());
  for (int i = 0; i < lastid(); i++) {
    if (i == id()) continue;
    msg->dst(i);
    std::lock_guard<std::mutex> l(moutbox);
    _outbox.push_back(msg);
  }
};
}
}
