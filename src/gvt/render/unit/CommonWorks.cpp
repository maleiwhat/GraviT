#include "gvt/render/unit/CommonWorks.h"

#include <iostream>

namespace gvt {
namespace render {
namespace unit {

STATIC_WORK_TAG(Command)
STATIC_WORK_TAG(PingTest)

bool Command::Action(Worker* worker) {
  int type = GetType();
  if (type == QUIT) {
#ifndef NDEBUG
    std::cout << "rank " << worker->GetRank() << " quitting communicator in "
              << __PRETTY_FUNCTION__ << std::endl;
#endif
    worker->QuitCommunicator();
  }

  return true;
}

bool PingTest::Action(Worker* worker) {
  Work* work = NULL;
  int rank = worker->GetRank();
  Communicator* comm = worker->GetCommunicator();

  std::cout << "rank " << rank << " got PingTest" << std::endl;

  if (rank == worker->GetMpiSize() - 1) {
    std::cout << "rank " << rank << " broadcasting quit message" << std::endl;
    work = new Command(Command::QUIT);
    work->SendAll(comm);
  } else {
    work = new PingTest(rank);
    work->Send(rank + 1, comm);
  }

  return true;
}

}  // namespace unit
}  // namespace render
}  // namespace gvt


