/* =======================================================================================
   This file is released as part of GraviT - scalable, platform independent ray tracing
   tacc.github.io/GraviT

   Copyright 2013-2015 Texas Advanced Computing Center, The University of Texas at Austin
   All rights reserved.

   Licensed under the BSD 3-Clause License, (the "License"); you may not use this file
   except in compliance with the License.
   A copy of the License is included with this software in the file LICENSE.
   If your copy does not contain the License, you may obtain a copy of the License at:

       http://opensource.org/licenses/BSD-3-Clause

   Unless required by applicable law or agreed to in writing, software distributed under
   the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
   KIND, either express or implied.
   See the License for the specific language governing permissions and limitations under
   limitations under the License.

   GraviT is funded in part by the US National Science Foundation under awards ACI-1339863,
   ACI-1339881 and ACI-1339840
   ======================================================================================= */
//
// RayTracer.h
//
#ifndef GVT_RENDER_UNIT_RAY_TRACER_H
#define GVT_RENDER_UNIT_RAY_TRACER_H

#include "gvt/render/unit/Types.h"
#include "gvt/render/unit/Profiler.h"

#include <cassert>

namespace gvt {
namespace render {
namespace unit {

class Work;
class Worker;
class Voter;
class Communicator;

class RayTracer {
 public:
  RayTracer(const MpiInfo& mpi, Worker* worker, Communicator* comm)
      : mpiInfo(mpi), worker(worker), comm(comm) {}
  virtual ~RayTracer() {}

  virtual void Trace() = 0;
  virtual Voter* GetVoter() { return NULL; }
  virtual void CompositeFrameBuffers() { assert(false); }

  // helper functions
  // TODO: avoid this (tbd)
  // virtual void CopyRays(const RayBuffer& ray_buffer) { assert(false); }

  virtual void EnqueWork(Work* work) { assert(false); }
  virtual bool IsRayQueueEmpty() const {
    assert(false);
    return false;
  }
  virtual profiler::Profiler *GetProfiler() {
    assert(false);
    return nullptr;
  }

  virtual void CompositeFramebuffers() { assert(false); }
  virtual void SignalCompositeDone() { assert(false); }

 protected:
  const MpiInfo mpiInfo;
  Worker* worker;
  Communicator* comm;
};

}  // namespace unit
}  // namespace render
}  // namespace gvt

#endif
