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

ImageTracer::ImageTracer() : RayTracer() { _queue.setQueuePolicy<LargestQueueFirst>(); }

ImageTracer::~ImageTracer() {}

void ImageTracer::operator()() {
  std::shared_ptr<gvt::comm::communicator> comm = gvt::comm::communicator::singleton();
  gvt::render::RenderContext &cntxt = *gvt::render::RenderContext::instance();
  gvt::core::DBNodeH rootnode = cntxt.getRootNode();
  size_t width = cntxt.getRootNode()["Film"]["width"].value().toInteger();
  size_t height = cntxt.getRootNode()["Film"]["height"].value().toInteger();
  size_t adapterType = cntxt.getRootNode()["Schedule"]["adapter"].value().toInteger();

  std::shared_ptr<gvt::render::data::scene::gvtCameraBase> _cam = cntxt.getCamera();
  std::shared_ptr<gvt::render::composite::ImageComposite> composite_buffer =
      cntxt.getComposite<gvt::render::composite::ImageComposite>();
  GVT_ASSERT(composite_buffer,
             "Invalid image composite buffer, please instanciate a proper "
             "composite buffer in context");
  std::shared_ptr<gvt::render::data::scene::Image> image = cntxt.getImage();
  // std::shared_ptr<gvt::render::data::scene::Image> _img = cntxt.getImage();

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

  int ray_portion = _cam->rays.size() / comm->lastid();
  int rays_start = comm->id() * ray_portion;
  size_t rays_end =
      (comm->id() + 1) == comm->lastid()
          ? _cam->rays.size()
          : (comm->id() + 1) * ray_portion; // tack on any odd rays to last proc

  gvt::render::actor::RayVector lrays;
  lrays.assign(_cam->rays.begin() + rays_start, _cam->rays.begin() + rays_end);
  _cam->rays.clear();

  processRayQueue(lrays);

  bool GlobalFrameFinished = false;

  while (!_queue.empty()) {
    int target = -1;
    gvt::render::actor::RayVector toprocess, moved_rays;
    _queue.dequeue(target, toprocess);
    if (target != -1) {
      trace(target, meshRef[target], toprocess, moved_rays, instM[target],
            instMinv[target], instMinvN[target], lights);

      processRayQueue(moved_rays, target);
    }
  }
  // Start composite
  float *img_final = composite_buffer->composite();
};

// bool ImageTracer::MessageManager(std::shared_ptr<gvt::comm::Message> msg) {
//   return Tracer::MessageManager(msg);
// }

void ImageTracer::updateGeometry() {}
}
}
