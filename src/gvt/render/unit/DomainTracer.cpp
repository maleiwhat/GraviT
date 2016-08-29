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
// DomainTracer.cpp
//

#include "gvt/render/unit/DomainTracer.h"

#include <pthread.h>
#include <cstring>
#include <iostream>

#include "gvt/render/unit/Communicator.h"

#include "gvt/render/unit/CommonWorks.h"
#include "gvt/render/unit/DomainWorks.h"
#include "gvt/render/unit/Voter.h"

#include "gvt/render/actor/Ray.h"

#include <gvt/core/mpi/Wrapper.h>
#include <gvt/render/Types.h>
#ifdef GVT_USE_MPE
#include "mpe.h"
#endif
#include <gvt/render/RenderContext.h>
#include <gvt/render/Schedulers.h>
#include <gvt/render/Types.h>
#include <gvt/render/algorithm/TracerBase.h>

#ifdef GVT_RENDER_ADAPTER_EMBREE
#include <gvt/render/adapter/embree/Wrapper.h>
#endif

#ifdef GVT_RENDER_ADAPTER_MANTA
#include <gvt/render/adapter/manta/Wrapper.h>
#endif
#ifdef GVT_RENDER_ADAPTER_OPTIX
#include <gvt/render/adapter/optix/Wrapper.h>
#endif
#if defined(GVT_RENDER_ADAPTER_OPTIX) && defined(GVT_RENDER_ADAPTER_EMBREE)
#include <gvt/render/adapter/heterogeneous/Wrapper.h>
#endif

// #include <boost/foreach.hpp>

#include <set>

// #define VERIFY
// #define DEBUG_TX

namespace gvt {
namespace render {
namespace unit {

using namespace gvt::render::actor;
using namespace gvt::render::data::scene;
using namespace gvt::render::unit::profiler;

// gvt::core::time::timer t_send(false, "domain tracer: send :");
// gvt::core::time::timer t_recv(false, "domain tracer: recv :");
// gvt::core::time::timer t_vote(false, "domain tracer: vote :");

DomainTracer::DomainTracer(const MpiInfo &mpiInfo, Worker *worker, Communicator *comm, RayVector &rays,
                           gvt::render::data::scene::Image &image)
    : RayTracer(mpiInfo, worker, comm), AbstractTrace(rays, image), numInFlightRays(0) {
  voter = NULL;
  if (mpiInfo.size > 1) voter = new Voter(mpiInfo, *this, comm);

  // pthread_mutex_init(&workQ_mutex, NULL);
  pthread_mutex_init(&rayMsgQ_mutex, NULL);
  pthread_mutex_init(&voteMsgQ_mutex, NULL);
#ifdef GVT_USE_MPE
  // MPI_Comm_rank(MPI_COMM_WORLD, &rank);
  MPE_Log_get_state_eventIDs(&tracestart, &traceend);
  MPE_Log_get_state_eventIDs(&shufflestart, &shuffleend);
  MPE_Log_get_state_eventIDs(&framebufferstart, &framebufferend);
  MPE_Log_get_state_eventIDs(&localrayfilterstart, &localrayfilterend);
  MPE_Log_get_state_eventIDs(&intersectbvhstart, &intersectbvhend);
  MPE_Log_get_state_eventIDs(&marchinstart, &marchinend);
  if (mpiInfo.rank == 0) {
    MPE_Describe_state(tracestart, traceend, "Process Queue", "blue");
    MPE_Describe_state(shufflestart, shuffleend, "Shuffle Rays", "green");
    MPE_Describe_state(framebufferstart, framebufferend, "Gather Framebuffer", "orange");
    MPE_Describe_state(localrayfilterstart, localrayfilterend, "Filter Rays Local", "coral");
    MPE_Describe_state(intersectbvhstart, intersectbvhend, "Intersect BVH", "azure");
    MPE_Describe_state(marchinstart, marchinend, "March Ray in", "LimeGreen");
  }
#endif
  gvt::core::Vector<gvt::core::DBNodeH> dataNodes = rootnode["Data"].getChildren();

  // create a map of instances to mpi rank
  for (size_t i = 0; i < instancenodes.size(); i++) {
    gvt::core::DBNodeH meshNode = instancenodes[i]["meshRef"].deRef();

    size_t dataIdx = -1;
    for (size_t d = 0; d < dataNodes.size(); d++) {
      if (dataNodes[d].UUID() == meshNode.UUID()) {
        dataIdx = d;
        break;
      }
    }

    // NOTE: mpi-data(domain) assignment strategy
    // debug (owning every domains)
    size_t mpiNode = dataIdx % mpiInfo.size;
    // size_t mpiNode = mpiInfo.rank;

    GVT_DEBUG(DBG_ALWAYS, "[" << mpiInfo.rank << "] domain scheduler: instId: " << i << ", dataIdx: " << dataIdx
                              << ", target mpi node: " << mpiNode << ", world size: " << mpiInfo.size);
#ifdef DEBUG_TX
    std::cout << "[" << mpiInfo.rank << "] domain scheduler: instId: " << i << ", dataIdx: " << dataIdx
              << ", target mpi node: " << mpiNode << ", world size: " << mpiInfo.size << std::endl;
#endif
    GVT_ASSERT(dataIdx != -1, "domain scheduler: could not find data node");
    mpiInstanceMap[i] = mpiNode;
  }
}

void DomainTracer::shuffleDropRays(gvt::render::actor::RayVector &rays) {
  const size_t chunksize = MAX(2, rays.size() / (std::thread::hardware_concurrency() * 4));
  static gvt::render::data::accel::BVH &acc = *dynamic_cast<gvt::render::data::accel::BVH *>(acceleration);
  static tbb::simple_partitioner ap;
  tbb::parallel_for(tbb::blocked_range<gvt::render::actor::RayVector::iterator>(rays.begin(), rays.end(), chunksize),
                    [&](tbb::blocked_range<gvt::render::actor::RayVector::iterator> raysit) {
                      std::vector<gvt::render::data::accel::BVH::hit> hits =
                          acc.intersect<GVT_SIMD_WIDTH>(raysit.begin(), raysit.end(), -1);
                      std::map<int, gvt::render::actor::RayVector> local_queue;
                      for (size_t i = 0; i < hits.size(); i++) {
                        gvt::render::actor::Ray &r = *(raysit.begin() + i);
                        if (hits[i].next != -1) {
                          r.origin = r.origin + r.direction * (hits[i].t * 0.8f);
                          const bool inRank = mpiInstanceMap[hits[i].next] == mpi.rank;
                          if (inRank) local_queue[hits[i].next].push_back(r);
                        }
                      }
                      for (auto &q : local_queue) {
                        tbb::mutex::scoped_lock lock(qmutex);
                        queue[q.first].insert(queue[q.first].end(),
                                              std::make_move_iterator(local_queue[q.first].begin()),
                                              std::make_move_iterator(local_queue[q.first].end()));
                      }
                    },
                    ap);

  rays.clear();
}

inline void DomainTracer::FilterRaysLocally() { shuffleDropRays(rays); }

inline void DomainTracer::Trace() {
  GVT_DEBUG(DBG_ALWAYS, "domain scheduler: starting, num rays: " << rays.size());
  gvt::core::DBNodeH root = gvt::render::RenderContext::instance()->getRootNode();

  clearBuffer();
  int adapterType = root["Schedule"]["adapter"].value().toInteger();

  long domain_counter = 0;

// FindNeighbors();

// sort rays into queues
// note: right now throws away rays that do not hit any domain owned by the
// current
// rank
#ifdef GVT_USE_MPE
  MPE_Log_event(localrayfilterstart, 0, NULL);
#endif
  // t_filter.resume();
  profiler.Start(Profiler::SHUFFLE);
  FilterRaysLocally();
  profiler.Stop(Profiler::SHUFFLE);
// t_filter.stop();
#ifdef GVT_USE_MPE
  MPE_Log_event(localrayfilterend, 0, NULL);
#endif

  GVT_DEBUG(DBG_LOW, "tracing rays");

  // process domains until all rays are terminated
  bool all_done = false;
  int nqueue = 0;
  std::set<int> doms_to_send;
  int lastInstance = -1;
  // gvt::render::data::domain::AbstractDomain* dom = NULL;

  gvt::render::actor::RayVector moved_rays;
  moved_rays.reserve(1000);

  int instTarget = -1;
  size_t instTargetCount = 0;

  gvt::render::Adapter *adapter = 0;

  // std::cout << "rank " << mpiInfo.rank << " filterRaysLocally done\n";

  while (!all_done) {
    profiler.Start(Profiler::SCHEDULE);
    // process domain with most rays queued
    instTarget = -1;
    instTargetCount = 0;

    // t_sort.resume();
    GVT_DEBUG(DBG_ALWAYS, "image scheduler: selecting next instance, num queues: " << this->queue.size());
    // for (std::map<int, gvt::render::actor::RayVector>::iterator q =
    // this->queue.begin(); q != this->queue.end();
    //      ++q) {
    for (auto &q : queue) {
      const bool inRank = mpiInstanceMap[q.first] == mpiInfo.rank;
      if (inRank && q.second.size() > instTargetCount) {
        instTargetCount = q.second.size();
        instTarget = q.first;
      }
    }
    // t_sort.stop();
    profiler.Stop(Profiler::SCHEDULE);
    GVT_DEBUG(DBG_ALWAYS, "image scheduler: next instance: " << instTarget << ", rays: " << instTargetCount);

    if (instTarget >= 0) {
      profiler.AddQueueState(queue);

      // t_adapter.resume();
      profiler.Start(Profiler::ADAPTER);

      profiler.AddCounter(Profiler::PROCESS_RAY, instTargetCount);
      profiler.AddCounter(Profiler::VALID_SCHEDULE, 1);

      gvt::render::Adapter *adapter = 0;
      // gvt::core::DBNodeH meshNode =
      // instancenodes[instTarget]["meshRef"].deRef();

      gvt::render::data::primitives::Mesh *mesh = meshRef[instTarget];

      // TODO: Make cache generic needs to accept any kind of adpater

      // 'getAdapterFromCache' functionality
      auto it = adapterCache.find(mesh);
      if (it != adapterCache.end()) {
        adapter = it->second;
        profiler.AddCounter(Profiler::ADAPTER_HIT, 1);
      } else {
        adapter = 0;
        profiler.AddCounter(Profiler::ADAPTER_MISS, 1);
      }
      if (!adapter) {
        GVT_DEBUG(DBG_ALWAYS, "image scheduler: creating new adapter");
        switch (adapterType) {
#ifdef GVT_RENDER_ADAPTER_EMBREE
        case gvt::render::adapter::Embree:
          adapter = new gvt::render::adapter::embree::data::EmbreeMeshAdapter(mesh);
          break;
#endif
#ifdef GVT_RENDER_ADAPTER_MANTA
        case gvt::render::adapter::Manta:
          adapter = new gvt::render::adapter::manta::data::MantaMeshAdapter(mesh);
          break;
#endif
#ifdef GVT_RENDER_ADAPTER_OPTIX
        case gvt::render::adapter::Optix:
          adapter = new gvt::render::adapter::optix::data::OptixMeshAdapter(mesh);
          break;
#endif

#if defined(GVT_RENDER_ADAPTER_OPTIX) && defined(GVT_RENDER_ADAPTER_EMBREE)
        case gvt::render::adapter::Heterogeneous:
          adapter = new gvt::render::adapter::heterogeneous::data::HeterogeneousMeshAdapter(mesh);
          break;
#endif
        default:
          GVT_DEBUG(DBG_SEVERE, "image scheduler: unknown adapter type: " << adapterType);
        }

        adapterCache[mesh] = adapter;
      }
      profiler.Stop(Profiler::ADAPTER);

      // std::cout << "rank " << mpiInfo.rank << " adapter cache access done\n";

      // t_adapter.stop();
      GVT_ASSERT(adapter != nullptr, "image scheduler: adapter not set");
      // end getAdapterFromCache concept

      GVT_DEBUG(DBG_ALWAYS, "image scheduler: calling process queue");
      {
        // t_trace.resume();
        profiler.Start(Profiler::TRACE);
        moved_rays.reserve(this->queue[instTarget].size() * 10);
        // #ifdef GVT_USE_DEBUG
        //         boost::timer::auto_cpu_timer t("Tracing rays in adapter: %w\n");
        // #endif
        adapter->trace(this->queue[instTarget], moved_rays, instM[instTarget], instMinv[instTarget],
                       instMinvN[instTarget], lights);

        this->queue[instTarget].clear();

        profiler.Stop(Profiler::TRACE);
        // t_trace.stop();
      }
      // printf("rank %d adapter.trace done moved_rays %lu inst %d\n", mpiInfo.rank, moved_rays.size(), instTarget);

      GVT_DEBUG(DBG_ALWAYS, "image scheduler: marching rays");
      // t_shuffle.resume();
      profiler.Start(Profiler::SHUFFLE);
      shuffleRays(moved_rays, instTarget);
      moved_rays.clear();
      profiler.Stop(Profiler::SHUFFLE);
      // std::cout << "rank " << mpiInfo.rank << " shuffleRays done\n";
      // t_shuffle.stop();
    } else { // if (instTarget > 0)
      profiler.AddCounter(Profiler::INVALID_SCHEDULE, 1);
    }

    all_done = Communicate();
    // std::cout << "rank " << mpiInfo.rank << " communmicate done\n";
    // #ifndef NDEBUG
    //     if (IsRayQueueEmpty())
    //       std::cout << "rank " << mpiInfo.rank << " ray queue is empty"
    //                 << std::endl;
    // #endif
  } // while (!all_done)

// std::cout << "domain scheduler: select time: " << t_sort.format();
// std::cout << "domain scheduler: trace time: " << t_trace.format();
// std::cout << "domain scheduler: shuffle time: " << t_shuffle.format();
// std::cout << "domain scheduler: send time: " << t_send.format();

#ifdef VERIFY
  for (auto &smap : send_map) {
    int dest = smap.first;
    size_t num_rays = smap.second;
    std::cout << "rank " << mpiInfo.rank << " send (dest, count): " << dest << " , " << num_rays << std::endl;
  }
  for (auto &rmap : recv_map) {
    int src = rmap.first;
    size_t num_rays = rmap.second;
    std::cout << "rank " << mpiInfo.rank << " recv (src, count): " << src << " , " << num_rays << std::endl;
  }
#endif

// add colors to the framebuffer
#ifdef GVT_USE_MPE
  MPE_Log_event(framebufferstart, 0, NULL);
#endif
  // t_gather.resume();
  // // this->gatherFramebuffers(this->rays_end - this->rays_start);
  // if (mpiInfo.rank == 0) {
  //   Work *work = new Composite(0);  // 0 is a dummy value
  //   work->SendAll(comm);
  // }
  profiler.Start(Profiler::COMPOSITE);
  this->gatherFramebuffers(this->rays_end - this->rays_start);
  profiler.Stop(Profiler::COMPOSITE);
// profiler.Start(Profiler::COMPOSITE);
// CompositeFrameBuffers();
// profiler.Stop(Profiler::COMPOSITE);
#ifdef DEBUG_TX
  std::cout << "rank " << mpiInfo.rank << " composite done" << std::endl;
#endif
// t_gather.stop();
#ifdef GVT_USE_MPE
  MPE_Log_event(framebufferend, 0, NULL);
#endif

  // t_frame.stop();
  // t_all = t_sort + t_trace + t_shuffle + t_gather + t_adapter + t_filter;
  //         // t_send + t_recv + t_vote;
  // t_diff = t_frame - t_all;
  std::cout << "rank " << mpiInfo.rank << " trace done\n";
}

// inline void DomainTracer::RecvMessage() {
//   pthread_mutex_lock(&workQ_mutex);
//   while (!workQ.empty()) {
//     Work *work = workQ.front();
//     int tag = work->GetTag();
//
//     if (tag == RemoteRays::tag) { // handle rays message
//       RemoteRays *rays = static_cast<RemoteRays *>(work);
//       int tx_type = rays->GetTransferType();
//       if (tx_type == RemoteRays::Request) {
//         RecvRays(rays);
//       } else if (tx_type == RemoteRays::Grant) {
//         voter->updateRayRx(rays->GetNumRays());
//       } else {
//         assert(false);
//       }
//     } else if (tag == Vote::tag) { // handle vote message
//       Vote *vote = static_cast<Vote *>(work);
//       voter->updateState(vote->GetVoteType() /* receivedVoteType */, false /* checkDone */);
//     } else {
//       assert(false);
//     }
//     workQ.pop();
//     delete work;
//   }
//   pthread_mutex_unlock(&workQ_mutex);
// }

inline void DomainTracer::recvRayMsg() {
  pthread_mutex_lock(&rayMsgQ_mutex);
  while (!rayMsgQ.empty()) {
    Work *work = rayMsgQ.front();
    assert(work->GetTag() == RemoteRays::tag);

    RemoteRays *rays = static_cast<RemoteRays *>(work);
    int tx_type = rays->GetTransferType();
    if (tx_type == RemoteRays::Request) {
      recvRays(rays);
    } else if (tx_type == RemoteRays::Grant) {
      voter->updateRayRx(rays->GetNumRays());
    } else {
      assert(false);
    }

    rayMsgQ.pop();
    delete work;
  }
  pthread_mutex_unlock(&rayMsgQ_mutex);
}

inline void DomainTracer::recvVoteMsg() {
  pthread_mutex_lock(&voteMsgQ_mutex);
  while (!voteMsgQ.empty()) {
    Work *work = voteMsgQ.front();
    assert(work->GetTag() == Vote::tag);

    Vote *vote = static_cast<Vote *>(work);
    voter->updateState(vote->GetVoteType() /* receivedVoteType */, false /* checkDone */);

    voteMsgQ.pop();
    delete work;
  }
  pthread_mutex_unlock(&voteMsgQ_mutex);
}

bool DomainTracer::Communicate() {
  bool done = false;
  if (mpiInfo.size > 1) {
    profiler.Start(Profiler::SEND);
    sendRays();
    profiler.Stop(Profiler::SEND);
    // std::cout << "rank " << mpiInfo.rank << " send done\n";
    profiler.Start(Profiler::RECV);
    recvRayMsg();
    recvVoteMsg();
    profiler.Stop(Profiler::RECV);
    // std::cout << "rank " << mpiInfo.rank << " recv done\n";

    profiler.Start(Profiler::VOTE);
    done = voter->isDone();
    profiler.Stop(Profiler::VOTE);
    // std::cout << "rank " << mpiInfo.rank << " voter done\n";
  } else {
    profiler.Start(Profiler::VOTE);
    done = IsRayQueueEmpty();
    profiler.Stop(Profiler::VOTE);
  }
  return done;
}

void DomainTracer::sendRays() {
  std::size_t ray_count = 0;

  for (auto &q : queue) {
    int instance = q.first;
    RayVector &rays = q.second;
    assert(instance > -1 && instance < mpiInstanceMap.size());
    int owner_process = mpiInstanceMap[instance];
    size_t num_rays_to_send = rays.size();

    if (owner_process != mpiInfo.rank && num_rays_to_send > 0) {
      // printf("sending rays...\n");
      ray_count += num_rays_to_send;

      voter->updateRayTx(num_rays_to_send);

      RemoteRays::Header header;
      header.transfer_type = RemoteRays::Request;
      header.sender = mpiInfo.rank;
      header.instance = instance;
      header.num_rays = num_rays_to_send;

      RemoteRays *work = new RemoteRays(header, rays);

#ifdef DEBUG_TX
      printf("[SEND_RAYS] rank %d: sending %lu rays (%d bytes) instance %d to "
             "rank %d\n",
             mpiInfo.rank, num_rays_to_send, work->GetSize(), instance, owner_process);
#endif

      work->Send(owner_process, comm);

      // RemoteRays *work = new RemoteRays(RemoteRays::getSize(
      //     num_rays_to_send * sizeof(gvt::render::actor::Ray)));
      // work->setup(RemoteRays::Request, rank, instance, rays);
      // work.Send(owner_process);
      // SendWork(work, owner_process);

      rays.clear();
#ifdef DEBUG_TX
      printf("rank %d: sent %lu rays (%d bytes) instance %d to rank %d\n", mpiInfo.rank, num_rays_to_send,
             work->GetSize(), instance, owner_process);
#endif
#ifdef VERIFY
      if (send_map.empty()) {
        for (int i = 0; i < mpiInfo.size; ++i) {
          send_map[i] = 0;
        }
      }
      send_map[owner_process] += num_rays_to_send;
#endif
    }
  }
  if (ray_count > 0) profiler.AddCounter(Profiler::SEND_RAY, ray_count);
}

inline void DomainTracer::EnqueWork(Work *work) {
  int tag = work->GetTag();
  if (tag == RemoteRays::tag) {
    pthread_mutex_lock(&rayMsgQ_mutex);
    rayMsgQ.push(work);
    pthread_mutex_unlock(&rayMsgQ_mutex);
  } else if (tag == Vote::tag) {
    pthread_mutex_lock(&voteMsgQ_mutex);
    voteMsgQ.push(work);
    pthread_mutex_unlock(&voteMsgQ_mutex);
  } else {
    assert(false);
  }
  // pthread_mutex_lock(&workQ_mutex);
  // workQ.push(work);
  // pthread_mutex_unlock(&workQ_mutex);
}

void DomainTracer::recvRays(const RemoteRays *rays) {
  std::size_t ray_count = 0;

#ifdef VERIFY
  if (recv_map.empty()) {
    for (int i = 0; i < mpiInfo.size; ++i) {
      recv_map[i] = 0;
    }
  }
  recv_map[rays->GetSender()] += rays->GetNumRays();
#endif
  CopyRays(*rays);

  RemoteRays::Header header;
  header.transfer_type = RemoteRays::Grant;
  header.sender = mpiInfo.rank;
  header.instance = rays->GetInstance();
  header.num_rays = rays->GetNumRays();

  RemoteRays *grant = new RemoteRays(header);
  grant->Send(rays->GetSender(), comm);

  ray_count += header.num_rays;
#ifdef DEBUG_TX
  printf("rank %d: recved %d rays instance %d \n", mpiInfo.rank, rays->GetNumRays(), rays->GetInstance());
#endif

  if (ray_count > 0) profiler.AddCounter(Profiler::RECV_RAY, ray_count);
}

void DomainTracer::CopyRays(const RemoteRays &rays) {
  int instance = rays.GetInstance();
  int num_rays = rays.GetNumRays();

  const Ray *begin = reinterpret_cast<const Ray *>(rays.GetRayBuffer());
  const Ray *end = begin + rays.GetNumRays();

#ifdef DEBUG_TX
  printf("ray copy begin %p end %p instance %d num_rays %d\n", begin, end, instance, num_rays);
#endif

  //   if (queue.find(instance) != queue.end()) {
  //     RayVector &r = queue[instance];
  //     r.insert(r.end(), begin, end);
  //   } else {
  //     queue[instance] = RayVector();
  //     RayVector &r = queue[instance];
  //     r.insert(r.end(), begin, end);
  //   }

  for (int i = 0; i < num_rays; ++i) {
    // if (queue.find(instance) == queue.end()) {
    //   gvt::render::actor::RayVector rv;
    //   queue[instance] = rv;
    // }
    const Ray &rr = *(begin + i);
    queue[instance].push_back(rr);
  }
}

void DomainTracer::LocalComposite() {
  const size_t size = width * height;
  const size_t chunksize = MAX(2, size / (std::thread::hardware_concurrency() * 4));
  static tbb::simple_partitioner ap;
  tbb::parallel_for(tbb::blocked_range<size_t>(0, size, chunksize),
                    [&](tbb::blocked_range<size_t> chunk) {
                      for (size_t i = chunk.begin(); i < chunk.end(); i++) image.Add(i, colorBuf[i]);
                    },
                    ap);
}

void DomainTracer::CompositeFrameBuffers() {
  // #ifndef NDEBUG
  //   std::cout << "rank " << mpiInfo.rank << " start " << __PRETTY_FUNCTION__
  //             << std::endl;
  // #endif
  LocalComposite();
  // for (size_t i = 0; i < size; i++) image.Add(i, colorBuf[i]);

  // if (!mpiInfo) return;

  size_t size = width * height;
  unsigned char *rgb = image.GetBuffer();

  int rgb_buf_size = 3 * size;

  unsigned char *bufs = (mpiInfo.rank == 0) ? new unsigned char[mpiInfo.size * rgb_buf_size] : NULL;
  // MPI_Barrier(MPI_COMM_WORLD);
  MPI_Gather(rgb, rgb_buf_size, MPI_UNSIGNED_CHAR, bufs, rgb_buf_size, MPI_UNSIGNED_CHAR, 0, MPI_COMM_WORLD);
  if (mpiInfo.rank == 0) {
    const size_t chunksize = MAX(2, size / (std::thread::hardware_concurrency() * 4));
    static tbb::simple_partitioner ap;
    tbb::parallel_for(tbb::blocked_range<size_t>(0, size, chunksize), [&](tbb::blocked_range<size_t> chunk) {

      for (int j = chunk.begin() * 3; j < chunk.end() * 3; j += 3) {
        for (size_t i = 1; i < mpiInfo.size; ++i) {
          int p = i * rgb_buf_size + j;
          // assumes black background, so adding is fine
          // (r==g==b== 0)
          rgb[j + 0] += bufs[p + 0];
          rgb[j + 1] += bufs[p + 1];
          rgb[j + 2] += bufs[p + 2];
        }
      }
    });
  }
  delete[] bufs;
  // #ifndef NDEBUG
  //   std::cout << "rank " << mpiInfo.rank << " done " << __PRETTY_FUNCTION__
  //             << std::endl;
  // #endif
}

} // namespace unit
} // namespace render
} // namespace gvt

