#include "scomm.h"
#include <cassert>
#include <iostream>
#include <memory>
#include <mpi.h>

namespace gvt {
namespace comm {
scomm::scomm() {}
void scomm::init(int argc, char *argv[]) {
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

  communicator::_instance = std::make_shared<scomm>();
  communicator::init(argc, argv);
}

void scomm::run() {
  while (!_terminate) {
    {
      MPI_Status status;
      int flag;
      MPI_Iprobe(MPI_ANY_SOURCE, CONTROL_SYSTEM_TAG, MPI_COMM_WORLD, &flag, &status);
      int n_bytes;
      MPI_Get_count(&status, MPI_BYTE, &n_bytes);

      if (n_bytes > 0) {
        int sender = status.MPI_SOURCE;

        const int data_size = n_bytes - sizeof(Message::header);

        std::shared_ptr<Message> msg = std::make_shared<Message>(data_size);

        std::cout << "Recv : " << n_bytes << " on " << id() << " from " << sender
                  << std::flush << std::endl;

        MPI_Recv(msg->getMessage<void>(), n_bytes, MPI_BYTE, MPI_ANY_SOURCE,
                 CONTROL_SYSTEM_TAG, MPI_COMM_WORLD, MPI_STATUS_IGNORE);
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
