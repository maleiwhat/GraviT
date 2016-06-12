#include "gvt/render/unit/CommonWorks.h"

#include <iostream>

namespace gvt {
namespace render {
namespace unit {

STATIC_WORK_TAG(Command)
STATIC_WORK_TAG(PingTest)

void Command::Action(Worker* worker) {
  int type = GetType();
  if (type == QUIT) {
    worker->SetDone();
  }
}

void PingTest::Action(Worker* worker) {
  Work* work = NULL;
  int rank = worker->GetRank();

  std::cout << "rank " << rank << " received PingTest\n";

  if (rank == worker->GetMpiSize() - 1) {
    work = new Command(Command::QUIT);
    work->SendAll(worker);
  } else {
    work = new PingTest(rank);
    work->Send(rank + 1, worker);
  }
}

}  // namespace unit
}  // namespace render
}  // namespace gvt


