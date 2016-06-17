#include "gvt/render/unit/DomainTracer.h"

#include <pthread.h>
#include <cstring>
#include <iostream>

#include "gvt/render/unit/Communicator.h"

#include "gvt/render/unit/CommonWorks.h"
#include "gvt/render/unit/DomainWorks.h"
#include "gvt/render/unit/TpcVoter.h"

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

namespace gvt {
namespace render {
namespace unit {

using namespace gvt::render::actor;
using namespace gvt::render::data::scene;
using namespace gvt::render::unit::profiler;

//gvt::core::time::timer t_send(false, "domain tracer: send :");
//gvt::core::time::timer t_recv(false, "domain tracer: recv :");
//gvt::core::time::timer t_vote(false, "domain tracer: vote :");

DomainTracer::DomainTracer(const MpiInfo &mpiInfo, Worker *worker,
                           Communicator *comm, RayVector &rays,
                           gvt::render::data::scene::Image &image)
    : RayTracer(mpiInfo, worker, comm), AbstractTrace(rays, image) {
  voter = NULL;
  if (mpiInfo.size > 1) voter = new TpcVoter(mpiInfo, *this, comm, worker);

  pthread_mutex_init(&workQ_mutex, NULL);
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
    MPE_Describe_state(framebufferstart, framebufferend, "Gather Framebuffer",
                       "orange");
    MPE_Describe_state(localrayfilterstart, localrayfilterend,
                       "Filter Rays Local", "coral");
    MPE_Describe_state(intersectbvhstart, intersectbvhend, "Intersect BVH",
                       "azure");
    MPE_Describe_state(marchinstart, marchinend, "March Ray in", "LimeGreen");
  }
#endif
  gvt::core::Vector<gvt::core::DBNodeH> dataNodes =
      rootnode["Data"].getChildren();

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
    size_t mpiNode = dataIdx % mpiInfo.size;

    GVT_DEBUG(DBG_ALWAYS, "[" << mpiInfo.rank << "] domain scheduler: instId: "
                              << i << ", dataIdx: " << dataIdx
                              << ", target mpi node: " << mpiNode
                              << ", world size: " << mpiInfo.size);

    GVT_ASSERT(dataIdx != -1, "domain scheduler: could not find data node");
    mpiInstanceMap[i] = mpiNode;
  }
}

void DomainTracer::shuffleDropRays(gvt::render::actor::RayVector &rays) {
  const size_t chunksize =
      MAX(2, rays.size() / (std::thread::hardware_concurrency() * 4));
  static gvt::render::data::accel::BVH &acc =
      *dynamic_cast<gvt::render::data::accel::BVH *>(acceleration);
  static tbb::simple_partitioner ap;
  tbb::parallel_for(
      tbb::blocked_range<gvt::render::actor::RayVector::iterator>(
          rays.begin(), rays.end(), chunksize),
      [&](tbb::blocked_range<gvt::render::actor::RayVector::iterator> raysit) {
        std::vector<gvt::render::data::accel::BVH::hit> hits =
            acc.intersect<GVT_SIMD_WIDTH>(raysit.begin(), raysit.end(), -1);
        std::map<int, gvt::render::actor::RayVector> local_queue;
        for (size_t i = 0; i < hits.size(); i++) {
          gvt::render::actor::Ray &r = *(raysit.begin() + i);
          if (hits[i].next != -1) {
            r.origin = r.origin + r.direction * (hits[i].t * 0.8f);
            const bool inRank = mpiInstanceMap[hits[i].next] == mpiInfo.rank;
            if (inRank) local_queue[hits[i].next].push_back(r);
          }
        }
        for (auto &q : local_queue) {
          queue_mutex[q.first].lock();
          queue[q.first].insert(
              queue[q.first].end(),
              std::make_move_iterator(local_queue[q.first].begin()),
              std::make_move_iterator(local_queue[q.first].end()));
          queue_mutex[q.first].unlock();
        }
      },
      ap);

  rays.clear();
}

inline void DomainTracer::FilterRaysLocally() { shuffleDropRays(rays); }

inline void DomainTracer::Trace() {
  GVT_DEBUG(DBG_ALWAYS,
            "domain scheduler: starting, num rays: " << rays.size());
  gvt::core::DBNodeH root =
      gvt::render::RenderContext::instance()->getRootNode();

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
  while (!all_done) {
    profiler.Start(Profiler::SCHEDULE);
    // process domain with most rays queued
    instTarget = -1;
    instTargetCount = 0;

    // t_sort.resume();
    GVT_DEBUG(DBG_ALWAYS,
              "image scheduler: selecting next instance, num queues: "
                  << this->queue.size());
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
    GVT_DEBUG(DBG_ALWAYS, "image scheduler: next instance: "
                              << instTarget << ", rays: " << instTargetCount);

    if (instTarget >= 0) {
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
            adapter =
                new gvt::render::adapter::embree::data::EmbreeMeshAdapter(mesh);
            break;
#endif
#ifdef GVT_RENDER_ADAPTER_MANTA
          case gvt::render::adapter::Manta:
            adapter =
                new gvt::render::adapter::manta::data::MantaMeshAdapter(mesh);
            break;
#endif
#ifdef GVT_RENDER_ADAPTER_OPTIX
          case gvt::render::adapter::Optix:
            adapter =
                new gvt::render::adapter::optix::data::OptixMeshAdapter(mesh);
            break;
#endif

#if defined(GVT_RENDER_ADAPTER_OPTIX) && defined(GVT_RENDER_ADAPTER_EMBREE)
          case gvt::render::adapter::Heterogeneous:
            adapter = new gvt::render::adapter::heterogeneous::data::
                HeterogeneousMeshAdapter(mesh);
            break;
#endif
          default:
            GVT_DEBUG(DBG_SEVERE,
                      "image scheduler: unknown adapter type: " << adapterType);
        }

        adapterCache[mesh] = adapter;
      }
      profiler.Stop(Profiler::ADAPTER);
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
        adapter->trace(this->queue[instTarget], moved_rays, instM[instTarget],
                       instMinv[instTarget], instMinvN[instTarget], lights);

        this->queue[instTarget].clear();

        profiler.Stop(Profiler::TRACE);
        // t_trace.stop();
      }

      GVT_DEBUG(DBG_ALWAYS, "image scheduler: marching rays");
      // t_shuffle.resume();
      profiler.Start(Profiler::SHUFFLE);
      shuffleRays(moved_rays, instTarget);
      moved_rays.clear();
      profiler.Stop(Profiler::SHUFFLE);
      // t_shuffle.stop();
    } else { // if (instTarget > 0)
      profiler.AddCounter(Profiler::INVALID_SCHEDULE, 1);
    }
   
    all_done = TransferRays();
  } // while (!all_done)

// std::cout << "domain scheduler: select time: " << t_sort.format();
// std::cout << "domain scheduler: trace time: " << t_trace.format();
// std::cout << "domain scheduler: shuffle time: " << t_shuffle.format();
// std::cout << "domain scheduler: send time: " << t_send.format();

#ifdef VERIFY
  for (auto &smap : send_map) {
    int dest = smap.first;
    size_t num_rays = smap.second;
    std::cout << "rank " << mpiInfo.rank << " send (dest, count): " << dest
              << " , " << num_rays << std::endl;
  }
  for (auto &rmap : recv_map) {
    int src = rmap.first;
    size_t num_rays = rmap.second;
    std::cout << "rank " << mpiInfo.rank << " recv (src, count): " << src
              << " , " << num_rays << std::endl;
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
  CompositeFrameBuffers();
  profiler.Stop(Profiler::COMPOSITE);
  // t_gather.stop();
#ifdef GVT_USE_MPE
  MPE_Log_event(framebufferend, 0, NULL);
#endif

  profiler.Start(Profiler::VOTE);
  if (voter) {
    voter->reset();
  }
  profiler.Stop(Profiler::VOTE);

  // t_frame.stop();
  // t_all = t_sort + t_trace + t_shuffle + t_gather + t_adapter + t_filter;
  //         // t_send + t_recv + t_vote;
  // t_diff = t_frame - t_all;
}

bool DomainTracer::TransferRays() {
  bool done;
  if (mpiInfo.size > 1) {
    profiler.Start(Profiler::VOTE);
    bool allowed = voter->isCommunicationAllowed();
    profiler.Stop(Profiler::VOTE);

    if (allowed) {  // TODO: potential improvement
      // t_send.resume();
      profiler.Start(Profiler::SEND);
      SendRays();
      profiler.Stop(Profiler::SEND);
      // t_send.stop();
      // profiler.update(Profiler::Send, t_send.getElapsed());

      // t_recv.resume();
      profiler.Start(Profiler::RECV);
      RecvRays();
      profiler.Stop(Profiler::RECV);
      // t_recv.stop();
      // profiler.update(Profiler::Receive, t_receive.getElapsed());
    }

    // t_vote.resume();
    profiler.Start(Profiler::VOTE);
    done = voter->updateState();
    profiler.Stop(Profiler::VOTE);
    // t_vote.stop();
    // profiler.update(Profiler::Vote, t_vote.getElapsed());

  } else {
    profiler.Start(Profiler::VOTE);
    done = IsDone();
    profiler.Stop(Profiler::VOTE);
  }
  // assert(!done || (done && !hasWork()));
  assert(!done || (done && IsDone()));
  return done;
}

void DomainTracer::SendRays() {
  std::size_t ray_count = 0;

  for (auto &q : queue) {
    int instance = q.first;
    RayVector &rays = q.second;
    int owner_process = mpiInstanceMap[instance];
    size_t num_rays_to_send = rays.size();

    if (owner_process != mpiInfo.rank && num_rays_to_send > 0) {
      ray_count += num_rays_to_send;

      voter->addNumPendingRays(num_rays_to_send);

      RemoteRays::Header header;
      header.transfer_type = RemoteRays::Request;
      header.sender = mpiInfo.rank;
      header.instance = instance;
      header.num_rays = num_rays_to_send;

      RemoteRays *work = new RemoteRays(header, rays);
      work->Send(owner_process, comm);

      // RemoteRays *work = new RemoteRays(RemoteRays::getSize(
      //     num_rays_to_send * sizeof(gvt::render::actor::Ray)));
      // work->setup(RemoteRays::Request, rank, instance, rays);
      // work.Send(owner_process);
      // SendWork(work, owner_process);

      rays.clear();
#ifndef NDEBUG
      printf("rank %d: sent %lu rays (%d bytes) instance %d to rank %d\n",
             mpiInfo.rank, num_rays_to_send, work->GetSize(), instance,
             owner_process);
#endif
#ifdef VERIFY
       if (send_map.empty()) {
         for (int i=0; i<mpiInfo.size; ++i) {
           send_map[i] = 0;
         }
       }
       send_map[owner_process] += num_rays_to_send; 
#endif
    }
  }
  if (ray_count > 0) profiler.AddCounter(Profiler::SEND_RAY, ray_count);
}

inline void DomainTracer::BufferWork(Work *work) {
  pthread_mutex_lock(&workQ_mutex);
  workQ.push(work);
  pthread_mutex_unlock(&workQ_mutex);
}

void DomainTracer::RecvRays() {
  std::size_t ray_count = 0;

  pthread_mutex_lock(&workQ_mutex);
  while (!workQ.empty()) {
    RemoteRays *rays = static_cast<RemoteRays *>(workQ.front());
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
#ifndef NDEBUG
    printf("rank %d: recved %d rays instance %d \n", mpiInfo.rank,
           rays->GetNumRays(), rays->GetInstance());
#endif
    workQ.pop();
    delete rays;
  }
  pthread_mutex_unlock(&workQ_mutex);

  if (ray_count > 0) profiler.AddCounter(Profiler::RECV_RAY, ray_count);
}

void DomainTracer::CopyRays(const RemoteRays &rays) {
  int instance = rays.GetInstance();
  int num_rays = rays.GetNumRays();

  const Ray *begin = reinterpret_cast<const Ray *>(rays.GetRayBuffer());
  const Ray *end = begin + rays.GetNumRays();

#ifndef NDEBUG
  printf("ray copy begin %p end %p instance %d num_rays %d\n", begin, end,
         instance, num_rays);
#endif

  if (queue.find(instance) != queue.end()) {
    RayVector &r = queue[instance];
    r.insert(r.end(), begin, end);
  } else {
    queue[instance] = RayVector();
    RayVector &r = queue[instance];
    r.insert(r.end(), begin, end);
  }
}

void DomainTracer::LocalComposite() {
  const size_t size = width * height;
  const size_t chunksize =
      MAX(2, size / (std::thread::hardware_concurrency() * 4));
  static tbb::simple_partitioner ap;
  tbb::parallel_for(tbb::blocked_range<size_t>(0, size, chunksize),
                    [&](tbb::blocked_range<size_t> chunk) {
                      for (size_t i = chunk.begin(); i < chunk.end(); i++)
                        image.Add(i, colorBuf[i]);
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

  unsigned char *bufs = (mpiInfo.rank == 0)
                            ? new unsigned char[mpiInfo.size * rgb_buf_size]
                            : NULL;
  // MPI_Barrier(MPI_COMM_WORLD);
  MPI_Gather(rgb, rgb_buf_size, MPI_UNSIGNED_CHAR, bufs, rgb_buf_size,
             MPI_UNSIGNED_CHAR, 0, MPI_COMM_WORLD);
  if (mpiInfo.rank == 0) {
    const size_t chunksize =
        MAX(2, size / (std::thread::hardware_concurrency() * 4));
    static tbb::simple_partitioner ap;
    tbb::parallel_for(tbb::blocked_range<size_t>(0, size, chunksize),
                      [&](tbb::blocked_range<size_t> chunk) {

                        for (int j = chunk.begin() * 3; j < chunk.end() * 3;
                             j += 3) {
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

}  // namespace unit
}  // namespace render
}  // namespace gvt

