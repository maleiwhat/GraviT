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

#include "ImageTracer.h"

namespace gvt {
namespace tracer {

template <>
void RayQueueManager::dequeue<ImageTracer>(int &target,
                                           gvt::render::actor::RayVector &_raylist) {
  int id = -1;
  unsigned _total = 0;
  {
    std::lock_guard<std::mutex> _lock(_protect);
    for (const auto &q : _queue) {
      if (q.second.size() > _total) {
        id = q.first;
        _total = q.second.size();
      }
    }
  }

  target = id;
  if (id == -1) return;
  std::lock_guard<std::mutex> _lock(_protect);
  std::swap(_queue[id], _raylist);
  _queue.erase(id);
  return;
}

ImageTracer::ImageTracer() : Tracer() {}

ImageTracer::~ImageTracer() {}

void ImageTracer::operator()() {
  gvt::render::RenderContext &cntxt = *gvt::render::RenderContext::instance();
  gvt::core::DBNodeH rootnode = cntxt.getRootNode();

  std::shared_ptr<gvt::render::data::scene::gvtCameraBase> _cam = cntxt.getCamera();
  std::shared_ptr<gvt::render::data::scene::Image> _img = cntxt.getImage();

  std::vector<gvt::core::DBNodeH> instancenodes = rootnode["Instances"].getChildren();
  GVT_ASSERT(!instancenodes.empty(), "Calling a tracer over an empty domain set");

  // Build BVH
  global_bvh = std::make_shared<gvt::render::data::accel::BVH>(instancenodes);
  // Cache context to avoid expensive lookups
  std::map<int, gvt::render::data::primitives::Mesh *> meshRef;
  std::map<int, glm::mat4 *> instM;
  std::map<int, glm::mat4 *> instMinv;
  std::map<int, glm::mat3 *> instMinvN;
  std::vector<gvt::render::data::scene::Light *> lights;

  for (int i = 0; i < instancenodes.size(); i++) {
    meshRef[i] = (gvt::render::data::primitives::Mesh *)instancenodes[i]["meshRef"]
                     .deRef()["ptr"]
                     .value()
                     .toULongLong();
    instM[i] = (glm::mat4 *)instancenodes[i]["mat"].value().toULongLong();
    instMinv[i] = (glm::mat4 *)instancenodes[i]["matInv"].value().toULongLong();
    instMinvN[i] = (glm::mat3 *)instancenodes[i]["normi"].value().toULongLong();
  }

  auto lightNodes = rootnode["Lights"].getChildren();

  lights.reserve(2);

  for (auto lightNode : lightNodes) {
    auto color = lightNode["color"].value().tovec3();

    if (lightNode.name() == std::string("PointLight")) {
      auto pos = lightNode["position"].value().tovec3();
      lights.push_back(new gvt::render::data::scene::PointLight(pos, color));
    } else if (lightNode.name() == std::string("AmbientLight")) {
      lights.push_back(new gvt::render::data::scene::AmbientLight(color));
    } else if (lightNode.name() == std::string("AreaLight")) {
      auto pos = lightNode["position"].value().tovec3();
      auto normal = lightNode["normal"].value().tovec3();
      auto width = lightNode["width"].value().toFloat();
      auto height = lightNode["height"].value().toFloat();
      lights.push_back(
          new gvt::render::data::scene::AreaLight(pos, color, normal, width, height));
    }
  }

  _cam->AllocateCameraRays();
  _cam->generateRays();
  _img->clear();

  processRayQueue(_cam->rays);

  bool GlobalFrameFinished = false;
  while (!GlobalFrameFinished) {
    if (!_queue.empty()) {
      int target = -1;
      gvt::render::actor::RayVector toprocess;
      _queue.dequeue<ImageTracer>(target, toprocess);
      if (target != -1) {
        // Check cache if adpter exists;
        // Call adapter
        // Process rays
      }
    }

    if (_queue.empty()) {
      // Ask if done;
      GlobalFrameFinished = true;
    }
  }
  // Start composite
};

bool ImageTracer::MessageManager(std::shared_ptr<gvt::comm::Message> msg) {
  return Tracer::MessageManager(msg);
}

void ImageTracer::processRayQueue(gvt::render::actor::RayVector &rays, const int src,
                                  const int dst) {
  if (dst >= 0) {
    _queue.enqueue(dst, rays);
    return;
  }

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
            // tbb::mutex::scoped_lock fbloc(colorBuf_mutex[r.id % width]);
            // colorBuf[r.id] += glm::vec4(r.color, r.w);
            // colorBuf[r.id][3] += r.w;
          }
        }
        for (auto &q : local_queue) {
          _queue.enqueue(q.first, local_queue[q.first]);
          // queue_mutex[q.first].lock();
          // queue[q.first].insert(queue[q.first].end(),
          //                       std::make_move_iterator(local_queue[q.first].begin()),
          //                       std::make_move_iterator(local_queue[q.first].end()));
          // queue_mutex[q.first].unlock();
        }
      },
      ap);

  // std::cout << "Finished shuffle" << std::endl;
  rays.clear();
}

void ImageTracer::updateGeometry() {}
}
}
