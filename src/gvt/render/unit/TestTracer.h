#ifndef GVT_RENDER_UNIT_TEST_TRACER_H
#define GVT_RENDER_UNIT_TEST_TRACER_H

#include "gvt/render/unit/Types.h"
#include "gvt/render/unit/RayTracer.h"
#include "gvt/render/unit/CommonWorks.h"

namespace gvt {
namespace render {
namespace unit {

class Worker;
class Communicator;

class PingTracer : public RayTracer {
 public:
  PingTracer(const MpiInfo& mpi, Worker* worker, Communicator* comm)
      : RayTracer(mpi, worker, comm) {}
  virtual ~PingTracer() {}

  virtual void Render() {
    Work* work = NULL;
    if (mpiInfo.size > 1) {
      int rank = mpiInfo.rank;
      if (rank == 0) {
        work = new PingTest(rank);
        work->Send(1, comm);
      }
    } else {
      std::cout << "need more than 1 rank to do ping test. Terminating.\n";
      work = new Command(Command::QUIT);
      work->SendAll(comm);
    }
  }
};

}  // namespace unit
}  // namespace render
}  // namespace gvt

#endif
