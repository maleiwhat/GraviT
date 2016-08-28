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
// DomainTracer.h
//

#ifndef GVT_RENDER_UNIT_DOMAIN_TRACER_H
#define GVT_RENDER_UNIT_DOMAIN_TRACER_H

#include <queue>
#include <pthread.h>
#include <set>

#include "gvt/render/unit/Types.h"
#include "gvt/render/unit/RayTracer.h"
#include "gvt/render/unit/Profiler.h"

#include "gvt/render/algorithm/TracerBase.h"
// #include "gvt/render/unit/AbstractTrace.h"
#include "gvt/render/actor/Ray.h"

// #define VERIFY

namespace gvt {
namespace render {
namespace data {
namespace scene {
class Image;
} // namespace scene
} // namespace data
} // namespace render
} // namespace gvt

namespace gvt {
namespace render {
namespace unit {

using namespace gvt::render::algorithm;
using namespace gvt::render::actor;
using namespace gvt::render::data::scene;

class Worker;
class Work;
class RemoteRays;
class Voter;
class Communicator;

/**
 * A domain tracer that communicates rays asynchronously.
 */
class DomainTracer : public RayTracer, public AbstractTrace {
public:
  DomainTracer(const MpiInfo &mpiInfo, Worker *worker, Communicator *comm, RayVector &rays,
               gvt::render::data::scene::Image &image);
  virtual ~DomainTracer() {}

  /**
   * The main rendering function that traces rays.
   */
  virtual void Trace();

  /**
   * A getter that returns the voter.
   */
  virtual Voter *GetVoter() { return voter; }

  /**
   * Push the incoming work sent by another node to the internal work queue (workQ).
   */
  virtual void EnqueWork(Work *work);

  /**
   * Check if all the ray queues are empty. This is used by the voter's updateState function.
   */
  virtual bool IsRayQueueEmpty() const {
    int busy = 0;
    for (auto &q : queue) busy += q.second.size();
    return (busy == 0);
  }

  /**
   * Composite frame buffers from all the MPI nodes using MPI_Gather.
   * This has been replaced by the Canvas class that uses an image compositer in IceT.
   */
  virtual void CompositeFrameBuffers();

  /**
   * A getter that returns the profiler.
   */
  profiler::Profiler &GetProfiler() { return profiler; }

private:
  void shuffleDropRays(gvt::render::actor::RayVector &rays);
  void FilterRaysLocally();

  // composite
  void LocalComposite();

  /**
   * The top-level communication function.
   */
  bool Communicate();

  /**
   * Send rays to the processes containing corresponding data.
   */
  void sendRays();

  /**
   * Receive incoming message from other nodes.
   */
  // void RecvMessage();
  void recvVoteMsg();
  void recvRayMsg();

  /**
   * Receive rays from other nodes.
   */
  void recvRays(const RemoteRays *rays);

  /**
   * Receive vote from other nodes.
   */
  // void RecvVote(const Work* work);

  void CopyRays(const RemoteRays &rays); // TODO: avoid this

  Voter *voter;

  /**
   * A mutex for the work queue.
   */
  // pthread_mutex_t workQ_mutex;
  pthread_mutex_t rayMsgQ_mutex;
  pthread_mutex_t voteMsgQ_mutex;

  /**
   * A work queue for registering all the incoming work sent by other nodes.
   * This DomainTracer class is responsible for deleting pointers in the queue.
   */
  // std::queue<Work *> workQ;
  std::queue<Work *> rayMsgQ;
  std::queue<Work *> voteMsgQ;

private:
  std::set<int> neighbors;

  size_t rays_start, rays_end;

  // caches meshes that are converted into the adapter's format
  std::map<gvt::render::data::primitives::Mesh *, gvt::render::Adapter *> adapterCache;
  std::map<int, size_t> mpiInstanceMap;
#ifdef GVT_USE_MPE
  int tracestart, traceend;
  int shufflestart, shuffleend;
  int framebufferstart, framebufferend;
  int localrayfilterstart, localrayfilterend;
  int intersectbvhstart, intersectbvhend;
  int marchinstart, marchinend;
#endif
#ifdef VERIFY
  std::map<int, size_t> send_map; // dest, num_rays
  std::map<int, size_t> recv_map; // src, num_rays
#endif
  /**
   * Number of inflight rays
   */
  int numInFlightRays;

  /**
   * A profiler for performance analysis.
   */
  profiler::Profiler profiler;
};

} // namespace unit
} // namespace render
} // namespace gvt

#endif
