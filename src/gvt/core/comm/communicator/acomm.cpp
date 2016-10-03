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
  int req = MPI_THREAD_MULTIPLE;
  int prov;
  MPI_Init_thread(&argc, &argv, req, &prov);

  switch (prov) {
  case MPI_THREAD_SINGLE:
    std::cout << "MPI_THREAD_SINGLE" << std::endl;
    break;
  case MPI_THREAD_FUNNELED:
    std::cout << "MPI_THREAD_FUNNELED" << std::endl;
    break;
  case MPI_THREAD_SERIALIZED:
    std::cout << "MPI_THREAD_SERIALIZED" << std::endl;
    break;
  case MPI_THREAD_MULTIPLE:
    std::cout << "MPI_THREAD_MULTIPLE" << std::endl;
    break;
  default:
    std::cout << "Upppsssss" << std::endl;
  }

  communicator::_instance = std::make_shared<acomm>();
  communicator::init(argc, argv);
}

void acomm::run() {
  while (!_terminate) {
    {

      if (!_outbox.empty()) {
        std::lock_guard<std::mutex> l(moutbox);
        std::shared_ptr<Message> msg = _outbox.front();
        _outbox.erase(_outbox.begin());
        MPI_Send(msg->getMessage<void>(), msg->buffer_size(), MPI::BYTE, msg->dst(),
                 CONTROL_SYSTEM_TAG, MPI_COMM_WORLD);
      }

      MPI_Status status;
      int flag;
      MPI_Iprobe(MPI_ANY_SOURCE, CONTROL_SYSTEM_TAG, MPI_COMM_WORLD, &flag, &status);
      int n_bytes;
      MPI_Get_count(&status, MPI_BYTE, &n_bytes);

      if (n_bytes > 0) {
        int sender = status.MPI_SOURCE;
        const auto data_size = n_bytes - sizeof(Message::header);
        std::shared_ptr<Message> msg = std::make_shared<Message>(data_size);

        std::cout << "Recv : " << n_bytes << " on " << id() << " from " << sender
                  << std::flush << std::endl;

        MPI_Recv(msg->getMessage<void>(), n_bytes, MPI_BYTE, sender, CONTROL_SYSTEM_TAG,
                 MPI_COMM_WORLD, MPI_STATUS_IGNORE);
        msg->size(data_size);
        std::lock_guard<std::mutex> l(minbox);
        if (msg->system_tag() == CONTROL_USER_TAG) _inbox.push_back(msg);
        if (msg->system_tag() == CONTROL_VOTE_TAG) voting->processMessage(msg);
      }
    }
  }
}

void acomm::send(std::shared_ptr<comm::Message> msg, std::size_t to) {
  std::cout << " ." << std::endl << std::flush;
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
