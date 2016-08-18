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

#include <gvt/render/Context.h>
#include <gvt/render/data/accel/BVH.h>

#include "ImageTracer.h"

namespace gvt {
namespace tracer {

template <>
gvt::render::actor::RayVector &&RayQueueManager::dequeue<ImageTracer>(int &target) {
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
  if (id == -1) return std::move(gvt::render::actor::RayVector());
  gvt::render::actor::RayVector _raylist = dequeue(id);

  std::lock_guard<std::mutex> _lock(_protect);
  _queue.erase(id);
  return std::move(_raylist);
}

ImageTracer::ImageTracer() : Tracer() {}

ImageTracer::~ImageTracer() {}

void ImageTracer::operator()() {
  gvt::render::RenderContext &cntxt = *gvt::render::RenderContext::instance();
  gvt::core::DBNodeH rootnode = cntxt.getRootNode();
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

  bool GlobalFrameFinished = false;
  gvt::render::RenderContext *cntx = gvt::render::RenderContext::instance();
  while (!GlobalFrameFinished) {
    if (!_queue.empty()) {
      int target = -1;
      gvt::render::actor::RayVector toprocess = _queue.dequeue(target);
      if (target != -1) {
        // Check cache if adpter exists;
        // Call adapter
        // Process rays
      }
    }
    if (_queue.empty()) {
      // Ask if done;
    }
  }
  // Start composite
};

bool ImageTracer::MessageManager(std::shared_ptr<gvt::comm::Message> msg) {
  return Tracer::MessageManager(msg);
}

void ImageTracer::processRayQueue(gvt::render::actor::RayVector &rays, const int src,
                                  const int dst) {}

void ImageTracer::updateGeometry() {}
}
}
