#ifndef GVT_RENDER_UNIT_RAY_TRACER_H
#define GVT_RENDER_UNIT_RAY_TRACER_H

#include "gvt/render/unit/Types.h"

#include <cassert>

namespace gvt {
namespace render {
namespace unit {

class Work;
class Worker;
class TpcVoter;
class Communicator;

class RayTracer {
 public:
  RayTracer(const MpiInfo& mpi, Worker* worker, Communicator* comm)
      : mpiInfo(mpi), worker(worker), comm(comm) {}
  virtual ~RayTracer() {}

  virtual void Render() = 0;
  virtual TpcVoter* GetVoter() { return NULL; }

  // helper functions
  // TODO: avoid this (tbd)
  // virtual void CopyRays(const RayBuffer& ray_buffer) { assert(false); }

  virtual void BufferWork(Work* work) { assert(false); }
  virtual bool IsDone() const {
    assert(false);
    return false;
  }

 protected:
  const MpiInfo mpiInfo;
  Worker* worker;
  Communicator* comm;
};

}  // namespace unit
}  // namespace render
}  // namespace gvt

#endif
