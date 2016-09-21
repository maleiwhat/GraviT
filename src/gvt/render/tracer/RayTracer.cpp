// clang-format off
/*
  =======================================================================================
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
   =======================================================================================

   */
// clang-format on

#include <tbb/blocked_range.h>
#include <tbb/mutex.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_for_each.h>
#include <tbb/partitioner.h>
#include <tbb/tick_count.h>

#include <gvt/render/Context.h>
#include <gvt/render/data/accel/BVH.h>

#include "RayTracer.h"

#include <gvt/render/composite/ImageComposite.h>
#include <gvt/render/Types.h>

#include <gvt/render/adapter/AdapterCache.h>
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

namespace gvt {
namespace tracer {

void RayTracer::trace(int target, gvt::render::data::primitives::Mesh *mesh,
                      gvt::render::actor::RayVector &rayList,
                      gvt::render::actor::RayVector &moved_rays, glm::mat4 *m,
                      glm::mat4 *minv, glm::mat3 *minvn,
                      std::vector<gvt::render::data::scene::Light *> &lights,
                      size_t begin, size_t end) {

  std::shared_ptr<gvt::comm::communicator> comm = gvt::comm::communicator::singleton();
  gvt::render::RenderContext &cntxt = *gvt::render::RenderContext::instance();
  gvt::core::DBNodeH rootnode = cntxt.getRootNode();
  size_t width = cntxt.getRootNode()["Film"]["width"].value().toInteger();
  size_t height = cntxt.getRootNode()["Film"]["height"].value().toInteger();
  size_t adapterType = cntxt.getRootNode()["Schedule"]["adapter"].value().toInteger();

  std::shared_ptr<gvt::render::Adapter> adapter = 0;

  if (_cache.keyExists(target)) {
    adapter = _cache.get(target);
  } else {
    GVT_DEBUG(DBG_ALWAYS, "image scheduler: creating new adapter");
    switch (adapterType) {
#ifdef GVT_RENDER_ADAPTER_EMBREE
    case gvt::render::adapter::Embree:
      adapter =
          std::make_shared<gvt::render::adapter::embree::data::EmbreeMeshAdapter>(mesh);
      break;
#endif
#ifdef GVT_RENDER_ADAPTER_MANTA
    case gvt::render::adapter::Manta:
      adapter =
          std::make_shared<gvt::render::adapter::manta::data::MantaMeshAdapter>(mesh);
      break;
#endif
#ifdef GVT_RENDER_ADAPTER_OPTIX
    case gvt::render::adapter::Optix:
      adapter =
          std::make_shared<gvt::render::adapter::optix::data::OptixMeshAdapter>(mesh);
      break;
#endif

#if defined(GVT_RENDER_ADAPTER_OPTIX) && defined(GVT_RENDER_ADAPTER_EMBREE)
    case gvt::render::adapter::Heterogeneous:
      adapter = std::make_shared<
          gvt::render::adapter::heterogeneous::data::HeterogeneousMeshAdapter>(mesh);
      break;
#endif
    default:
      GVT_DEBUG(DBG_SEVERE, "image scheduler: unknown adapter type: " << adapterType);
    }

    _cache.set(target, adapter);
    //    [mesh] = adapter;
  }
  adapter->trace(rayList, moved_rays, m, minv, minvn, lights);
};

void RayTracer::processRayQueue(gvt::render::actor::RayVector &rays, const int src,
                                const int dst) {
  if (dst >= 0) {
    _queue.enqueue(dst, rays);
    return;
  }

  gvt::render::RenderContext &cntxt = *gvt::render::RenderContext::instance();
  std::shared_ptr<gvt::render::composite::ImageComposite> composite_buffer =
      cntxt.getComposite<gvt::render::composite::ImageComposite>();

  const size_t chunksize =
      MAX(2, rays.size() / (std::thread::hardware_concurrency() * 4));
  // gvt::render::data::accel::BVH &acc = std::dynamic_casglobal_bvh;

  gvt::render::data::accel::BVH &acc =
      *dynamic_cast<gvt::render::data::accel::BVH *>(global_bvh.get());

  static tbb::simple_partitioner ap;
  tbb::parallel_for(
      tbb::blocked_range<gvt::render::actor::RayVector::iterator>(rays.begin(),
                                                                  rays.end(), chunksize),
      [&](tbb::blocked_range<gvt::render::actor::RayVector::iterator> raysit) {
        std::vector<gvt::render::data::accel::BVH::hit> hits =
            acc.intersect<GVT_SIMD_WIDTH>(raysit.begin(), raysit.end(), src);
        std::map<int, gvt::render::actor::RayVector> local_queue;
        for (size_t i = 0; i < hits.size(); i++) {
          gvt::render::actor::Ray &r = *(raysit.begin() + i);
          if (hits[i].next != -1) {
            r.origin = r.origin + r.direction * (hits[i].t * 0.95f);
            local_queue[hits[i].next].push_back(r);
          } else if (r.type == gvt::render::actor::Ray::SHADOW &&
                     glm::length(r.color) > 0) {
            composite_buffer->localAdd(r.id, r.color, r.w);
          }
        }
        for (auto &q : local_queue) {
          _queue.enqueue(q.first, local_queue[q.first]);
        }
      },
      ap);
};
}
}
