#include "scomm.h"
#include <cassert>
#include <memory>
#include <mpi.h>

namespace gvt {
namespace comm {
scomm::scomm() {}
void scomm::init(int argc, char *argv[]) {
  assert(!communicator::_instance);
  communicator::_instance = std::make_shared<scomm>();
  communicator::init(argc, argv);
}

void scomm::run() {
  while (!_terminate) {
    {
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
}
}
