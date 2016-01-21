/* =======================================================================================
   This file is released as part of GraviT - scalable, platform independent ray
   tracing
   tacc.github.io/GraviT

   Copyright 2013-2015 Texas Advanced Computing Center, The University of Texas
   at Austin
   All rights reserved.

   Licensed under the BSD 3-Clause License, (the "License"); you may not use
   this file
   except in compliance with the License.
   A copy of the License is included with this software in the file LICENSE.
   If your copy does not contain the License, you may obtain a copy of the
   License at:

       http://opensource.org/licenses/BSD-3-Clause

   Unless required by applicable law or agreed to in writing, software
   distributed under
   the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY
   KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under
   limitations under the License.

   GraviT is funded in part by the US National Science Foundation under awards
   ACI-1339863,
   ACI-1339881 and ACI-1339840
   =======================================================================================
   */

//
// TileWork.cpp
//

#include "gvt/render/unit/TileWork.h"
#include "gvt/render/unit/PixelWork.h"
#include "gvt/render/unit/RequestWork.h"
#include "gvt/render/unit/MpiRenderer.h"

#include <gvt/core/Types.h>
#include "gvt/core/mpi/Application.h"
#include "gvt/core/mpi/Work.h"
#include "gvt/core/math/Vector.h"
#include "gvt/core/DatabaseNode.h"

#include "gvt/render/RenderContext.h"
#include "gvt/render/Adapter.h"
#include "gvt/render/actor/Ray.h"
#include "gvt/render/data/primitives/BBox.h"
#include "gvt/render/data/accel/BVH.h"

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

#include <iostream>

#include <boost/foreach.hpp>
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/timer/timer.hpp>
#include <boost/range/algorithm.hpp>

#include <tbb/parallel_for_each.h>
#include <tbb/tick_count.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/mutex.h>

using namespace gvt::core::mpi;
using namespace gvt::core::math;
using namespace gvt::render::unit;
using namespace gvt::core;
using namespace gvt::render;
using namespace gvt::render::actor;
using namespace gvt::render::data::primitives;

#define DEBUG_TILE_WORK
// #define RENDER_MOSAIC_WITHOUT_TRACING

#ifdef RENDER_MOSAIC_WITHOUT_TRACING
#include <cstdlib>
#endif

WORK_CLASS(TileWork)

void TileWork::Serialize(size_t& size, unsigned char*& serialized) {

  size = 4 * sizeof(int);
  serialized = static_cast<unsigned char*>(malloc(size));

  unsigned char* buf = serialized;

  *reinterpret_cast<int*>(buf) = startX; buf += sizeof(int);
  *reinterpret_cast<int*>(buf) = startY; buf += sizeof(int);
  *reinterpret_cast<int*>(buf) = width;  buf += sizeof(int);
  *reinterpret_cast<int*>(buf) = height; buf += sizeof(int);
}

Work* TileWork::Deserialize(size_t size, unsigned char* serialized) {

  if (size != (4 * sizeof(int))) {
    std::cerr << "Test deserializer ctor with size != 4 * sizeof(int)\n";
    exit(1);
  }

  unsigned char* buf = serialized;
  TileWork* tileWork = new TileWork;

  tileWork->startX = *reinterpret_cast<int*>(buf); buf += sizeof(int);
  tileWork->startY = *reinterpret_cast<int*>(buf); buf += sizeof(int);
  tileWork->width  = *reinterpret_cast<int*>(buf); buf += sizeof(int);
  tileWork->height = *reinterpret_cast<int*>(buf); buf += sizeof(int);

  return static_cast<Work*>(tileWork);
}

void TileWork::setupAction() {
  renderer = static_cast<MpiRenderer*>(Application::GetApplication());
  framebuffer = renderer->getFramebuffer();
  acceleration = renderer->getAcceleration();
  queue_mutex = renderer->getQueueMutex();
  colorBuf_mutex = renderer->getColorBufMutex();
  root = RenderContext::instance()->getRootNode();
  imageWidth = variant_toInteger(root["Film"]["width"].value());
  imageHeight = variant_toInteger(root["Film"]["height"].value());
}

bool TileWork::Action() {

#ifdef DEBUG_TILE_WORK
  printf("Rank %d: start processing tile(%d, %d, %d, %d)\n",
          Application::GetApplication()->GetRank(),
          startX, startY, width, height);
#endif

  setupAction();

  RayVector rays;
  generatePrimaryRays(rays);

  traceRays(rays);

  sendRequest(gvt::render::unit::rank::Server);
  sendPixels(gvt::render::unit::rank::Display);

  return false;
}

void TileWork::setTileInfo(int x, int y, int w, int h,
                           std::vector<GVT_COLOR_ACCUM>* framebuffer) {
  startX = x;
  startY = y;
  width = w;
  height = h;
  this->framebuffer = framebuffer;
}

void TileWork::setTileSize(int x, int y, int w, int h) {
  startX = x;
  startY = y;
  width = w;
  height = h;
}

void TileWork::sendRequest(int rank) {
  RequestWork request;
  request.setSourceRank(Application::GetApplication()->GetRank());
  request.Send(rank);
}

void TileWork::sendPixels(int rank) {
  #ifdef DEBUG_TILE_WORK
  printf("Rank %d: sending pixels tile(%d %d %d %d) to Rank %d\n",
         Application::GetApplication()->GetRank(),
         startX, startY, width, height, rank);
  #endif
  PixelWork pixels;
  pixels.setTileInfo(startX, startY, width, height, framebuffer);
  pixels.Send(rank);
}

void TileWork::generatePrimaryRays(RayVector& rays) {

  int tileW = width;
  int tileH = height;

  Point4f eye = variant_toPoint4f(root["Camera"]["eyePoint"].value());
  float fov = variant_toFloat(root["Camera"]["fov"].value());
  const AffineTransformMatrix<float>& cameraToWorld =
      renderer->getCamera()->getCameraToWorld();

  rays.resize(tileW * tileH);

  // Generate rays direction in camera space and transform to world space.
  int i, j;
  int tilePixelId, imagePixelId;
  float aspectRatio = float(imageWidth) / float(imageHeight);
  float x, y;
  // these basis directions are scaled by the aspect ratio and
  // the field of view.
  Vector4f camera_vert_basis_vector =
      Vector4f(0, 1, 0, 0) * tan(fov * 0.5);

  Vector4f camera_horiz_basis_vector =
      Vector4f(1, 0, 0, 0) * tan(fov * 0.5) * aspectRatio;

  Vector4f camera_normal_basis_vector = Vector4f(0, 0, 1, 0);
  Vector4f camera_space_ray_direction;

  for (j = startY; j < startY + tileH; j++) {
    for (i = startX; i < startX + tileW; i++) {
      // select a ray and load it up
      tilePixelId = (j - startY) * tileW + (i - startX);
      imagePixelId = j * imageWidth + i;
      Ray &ray = rays[tilePixelId];
      ray.id = imagePixelId;
      ray.w = 1.0; // ray weight 1 for no subsamples. mod later
      ray.origin = eye;
      ray.type = Ray::PRIMARY;
      // calculate scale factors -1.0 < x,y < 1.0
      x = 2.0 * float(i) / float(imageWidth - 1) - 1.0;
      y = 2.0 * float(j) / float(imageHeight - 1) - 1.0;
      // calculate ray direction in camera space;
      camera_space_ray_direction = camera_normal_basis_vector +
                                   x * camera_horiz_basis_vector +
                                   y * camera_vert_basis_vector;
      // transform ray to world coordinate space;
      ray.setDirection(cameraToWorld * camera_space_ray_direction.normalize());
      ray.depth = 0;
    }
  }
}

void TileWork::filterRaysLocally(RayVector& rays) {
  auto nullNode = gvt::core::DBNodeH(); // temporary workaround until
                                        // shuffleRays is fully replaced
  GVT_DEBUG(DBG_ALWAYS,
            "image scheduler: filter locally non mpi: " << rays.size());
  shuffleRays(rays, nullNode);
}

/**
 * Given a queue of rays, intersects them against the accel structure
 * to find out what instance they will hit next
 */
void TileWork::shuffleRays(gvt::render::actor::RayVector &rays,
                           gvt::core::DBNodeH instNode) {
  GVT_DEBUG(DBG_ALWAYS, "[" << mpi.rank << "] Shuffle: start");
  GVT_DEBUG(DBG_ALWAYS, "[" << mpi.rank
                            << "] Shuffle: rays: " << rays.size());
  const size_t raycount = rays.size();
  const int domID =
      (instNode) ? gvt::core::variant_toInteger(instNode["id"].value()) : -1;
  const gvt::render::data::primitives::Box3D domBB =
      (instNode) ? *gvt::core::variant_toBox3DPtr(instNode["bbox"].value())
                 : gvt::render::data::primitives::Box3D();

  // tbb::parallel_for(size_t(0), size_t(rays.size()),
  //      [&] (size_t index) {
  tbb::parallel_for(
      tbb::blocked_range<gvt::render::actor::RayVector::iterator>(
          rays.begin(), rays.end()),
      [&](tbb::blocked_range<gvt::render::actor::RayVector::iterator>
              raysit) {
        std::map<int, gvt::render::actor::RayVector> local_queue;
        for (gvt::render::actor::Ray &r : raysit) {
        // for (gvt::render::actor::RayVector::iterator it = rays.begin(); it != rays.end(); ++it) {
        //   gvt::render::actor::Ray &r = *it;
          if (domID != -1) {
            float t = FLT_MAX;
            if (r.domains.empty() && domBB.intersectDistance(r, t)) {
              r.origin += r.direction * t;
            }
          }
          if (r.domains.empty()) {
            acceleration->intersect(r, r.domains);
            boost::sort(r.domains);
          }
          if (!r.domains.empty() && (int)(*r.domains.begin()) == domID) {
            r.domains.erase(r.domains.begin());
          }
          if (!r.domains.empty()) {
            int firstDomainOnList = (*r.domains.begin());
            r.domains.erase(r.domains.begin());
            // tbb::mutex::scoped_lock sl(queue_mutex[firstDomainOnList]);
            local_queue[firstDomainOnList].push_back(r);
          } else if (instNode) {
            assert(r.id < 307200 && r.id > -1);
            // TODO (hpark): do we need this lock? 
            tbb::mutex::scoped_lock fbloc(colorBuf_mutex[r.id % imageWidth]);
            GVT_COLOR_ACCUM& color = (*framebuffer)[r.id];
            for (int i = 0; i < 3; i++)
              color.rgba[i] += r.color.rgba[i];
            color.rgba[3] = 1.f;
            color.clamp();
          }
        }

        std::vector<int> _doms;
        std::transform(
            local_queue.begin(), local_queue.end(), std::back_inserter(_doms),
            [](const std::map<int, gvt::render::actor::RayVector>::value_type
                   &pair) { return pair.first; });
        while(!_doms.empty()) {
            int dom = _doms.front();
            _doms.erase(_doms.begin());
            if(queue_mutex[dom].try_lock()) {
              queue[dom].insert(queue[dom].end(),
                            std::make_move_iterator(local_queue[dom].begin()),
                            std::make_move_iterator(local_queue[dom].end()));
              queue_mutex[dom].unlock();
            } else {
              _doms.push_back(dom);
            }
        }
      });
  rays.clear();
}

void TileWork::traceRays(RayVector& rays) {

  #ifdef DEBUG_TILE_WORK
  printf("Rank %d: tracing rays using image scheduler\n",
         Application::GetApplication()->GetRank());
  #endif

  #ifdef RENDER_MOSAIC_WITHOUT_TRACING
  renderMosaic();
  #else

  gvt::core::Vector<DBNodeH>& instancenodes = renderer->getInstanceNodes();

  // boost::timer::cpu_timer t_sched;
  // t_sched.start();
  // boost::timer::cpu_timer t_trace;
  GVT_DEBUG(DBG_ALWAYS,
            "image scheduler: starting, num rays: " << rays.size());
  GVT_ASSERT((instancenodes.size() > 0),
             "image scheduler: instance list is null");
  int adapterType = variant_toInteger(root["Schedule"]["adapter"].value());
  // sort rays into queues
  filterRaysLocally(rays);
  RayVector moved_rays;
  int instTarget = -1, instTargetCount = 0;
  // process domains until all rays are terminated
  do {
    // process domain with most rays queued
    instTarget = -1;
    instTargetCount = 0;
    GVT_DEBUG(DBG_ALWAYS,
              "image scheduler: selecting next instance, num queues: "
                  << this->queue.size());
    for (std::map<int, RayVector>::iterator q = this->queue.begin();
         q != this->queue.end(); ++q) {
      if (q->second.size() > static_cast<size_t>(instTargetCount)) {
        instTargetCount = q->second.size();
        instTarget = q->first;
      }
    }
    GVT_DEBUG(DBG_ALWAYS, "image scheduler: next instance: "
                              << instTarget << ", rays: " << instTargetCount);
    if (instTarget >= 0) {
      Adapter *adapter = 0;
      DBNodeH meshNode = instancenodes[instTarget]["meshRef"].deRef();
      // TODO: Make cache generic needs to accept any kind of adpater
      // 'getAdapterFromCache' functionality
      auto it = adapterCache.find(meshNode.UUID());
      if (it != adapterCache.end()) {
        adapter = it->second;
        GVT_DEBUG(DBG_ALWAYS, "image scheduler: using adapter from cache["
                                  << gvt::core::uuid_toString(meshNode.UUID())
                                  << "], " << (void *)adapter);
      }
      if (!adapter) {
        GVT_DEBUG(DBG_ALWAYS, "image scheduler: creating new adapter");
        switch (adapterType) {
        #ifdef GVT_RENDER_ADAPTER_EMBREE
        case gvt::render::adapter::Embree:
          adapter = new gvt::render::adapter::embree::data::EmbreeMeshAdapter(
              meshNode);
          break;
        #endif
        #ifdef GVT_RENDER_ADAPTER_MANTA
        case gvt::render::adapter::Manta:
          adapter = new gvt::render::adapter::manta::data::MantaMeshAdapter(
              meshNode);
          break;
        #endif
        #ifdef GVT_RENDER_ADAPTER_OPTIX
        case gvt::render::adapter::Optix:
          adapter = new gvt::render::adapter::optix::data::OptixMeshAdapter(
              meshNode);
          break;
        #endif
        #if defined(GVT_RENDER_ADAPTER_OPTIX) && \
            defined(GVT_RENDER_ADAPTER_EMBREE)
        case gvt::render::adapter::Heterogeneous:
          adapter = new gvt::render::adapter::heterogeneous::data::HeterogeneousMeshAdapter(
              meshNode);
          break;
        #endif
        default:
          GVT_DEBUG(DBG_SEVERE,
                    "image scheduler: unknown adapter type: "
                    << adapterType);
        }
        adapterCache[meshNode.UUID()] = adapter;
      }
      GVT_ASSERT(adapter != nullptr, "image scheduler: adapter not set");
      // end getAdapterFromCache concept
      GVT_DEBUG(DBG_ALWAYS, "image scheduler: calling process queue");
      {
        // t_trace.resume();
        moved_rays.reserve(this->queue[instTarget].size() * 10);

        #ifdef GVT_USE_DEBUG
        boost::timer::auto_cpu_timer t("Tracing rays in adapter: %w\n");
        #endif

        adapter->trace(this->queue[instTarget], moved_rays,
                       instancenodes[instTarget]);
        this->queue[instTarget].clear();
        // t_trace.stop();
      }
      GVT_DEBUG(DBG_ALWAYS, "image scheduler: marching rays");
      shuffleRays(moved_rays, instancenodes[instTarget]);
      moved_rays.clear();
    }
  } while (instTarget != -1);

  GVT_DEBUG(DBG_ALWAYS,
            "image scheduler: adapter cache size: " << adapterCache.size());
  // std::cout << "image scheduler: trace time: " << t_trace.format();
  // std::cout << "image scheduler: sched time: " << t_sched.format();

  #endif // RENDER_MOSAIC_WITHOUT_TRACING
}

// void TileWork::traceRaysDomainScheduler(RayVector& rays) {

//   #ifdef DEBUG_TILE_WORK
//   printf("Rank %d: tracing rays using domain scheduler\n",
//          Application::GetApplication()->GetRank());
//   #endif

//   #ifdef RENDER_MOSAIC_WITHOUT_TRACING
//   renderMosaic();
//   #else
//   // TODO
//   #endif

// }

void TileWork::renderMosaic() {

  int rank = Application::GetApplication()->GetRank();

  int imagePixelId;
  int offsetR = (rand() % 100);
  int offsetG = (rand() % 100);
  int offsetB = (rand() % 100);
  for (int j = startY; j < startY + height; j++) {
    for (int i = startX; i < startX + width; i++) {
      imagePixelId = j * imageWidth + i;
      GVT_COLOR_ACCUM& color = (*framebuffer)[imagePixelId];
      color.rgba[0] = rank + offsetR;
      color.rgba[1] = rank + offsetG;
      color.rgba[2] = rank + offsetB;
    }
  }
}
