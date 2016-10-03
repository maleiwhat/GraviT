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

   GraviT is funded in part by the US National Science Foundation under awards
   ACI-1339863,
   ACI-1339881 and ACI-1339840
   =======================================================================================
   */

#include "DomainTracer.h"

#include <tbb/blocked_range.h>
#include <tbb/mutex.h>
#include <tbb/parallel_for.h>
#include <tbb/parallel_for_each.h>
#include <tbb/partitioner.h>
#include <tbb/tick_count.h>

#include <set>

#include <gvt/render/Context.h>
#include <gvt/render/Types.h>
#include <gvt/render/composite/ImageComposite.h>
#include <gvt/render/data/accel/BVH.h>

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

#include <gvt/core/utils/timer.h>
#include <gvt/render/tracer/Domain/Messages/SendRayList.h>

namespace gvt {
namespace tracer {

InNodeLargestQueueFirst::InNodeLargestQueueFirst() { refreshInNode(); }

void InNodeLargestQueueFirst::addToInNode(const int &id) { innode.push_back(id); }
void InNodeLargestQueueFirst::refreshInNode() {
  std::shared_ptr<gvt::comm::communicator> comm = gvt::comm::communicator::singleton();
  gvt::render::RenderContext &cntxt = *gvt::render::RenderContext::instance();
  gvt::core::DBNodeH rootnode = cntxt.getRootNode();
  std::vector<gvt::core::DBNodeH> instancenodes = rootnode["Instances"].getChildren();

  gvt::core::Vector<gvt::core::DBNodeH> dataNodes = rootnode["Data"].getChildren();
  std::map<int, std::set<std::string> > meshAvailbyMPI; // where meshes are by mpi node
  std::map<int, std::set<std::string> >::iterator lastAssigned; // instance-node
                                                                // round-robin assigment

  for (size_t i = 0; i < comm->lastid(); i++) meshAvailbyMPI[i].clear();

  // build location map, where meshes are by mpi node
  for (size_t i = 0; i < dataNodes.size(); i++) {
    std::vector<gvt::core::DBNodeH> locations = dataNodes[i]["Locations"].getChildren();
    for (auto loc : locations) {
      meshAvailbyMPI[loc.value().toInteger()].insert(dataNodes[i].UUID().toString());
    }
  }
  lastAssigned = meshAvailbyMPI.begin();
  // create a map of instances to mpi rank
  for (size_t i = 0; i < instancenodes.size(); i++) {
    mpiInstanceMap[i] = -1;
    if (instancenodes[i]["meshRef"].value().toUuid() != gvt::core::Uuid::null()) {
      gvt::core::DBNodeH meshNode = instancenodes[i]["meshRef"].deRef();
      // Instance to mpi-node Round robin assignment considering mesh availability
      auto startedAt = lastAssigned;
      do {
        if (lastAssigned->second.size() > 0) { // if mpi-node has no meshes, don't bother
          if (lastAssigned->second.find(meshNode.UUID().toString()) !=
              lastAssigned->second.end()) {
            mpiInstanceMap[i] = lastAssigned->first;
            lastAssigned++;
            if (lastAssigned == meshAvailbyMPI.end())
              lastAssigned = meshAvailbyMPI.begin();
            break;
          } else {
            // branch out from lastAssigned and search for a mpi-node with the mesh
            // keep lastAssigned to continue with round robin
            auto branchOutSearch = lastAssigned;
            do {
              if (branchOutSearch->second.find(meshNode.UUID().toString()) !=
                  branchOutSearch->second.end()) {
                mpiInstanceMap[i] = branchOutSearch->first;
                break;
              }
              branchOutSearch++;
              if (branchOutSearch == meshAvailbyMPI.end())
                branchOutSearch = meshAvailbyMPI.begin();
            } while (branchOutSearch != lastAssigned);
            break; // If the branch-out didn't found a node, break the main loop, meaning
                   // that no one has the mesh
          }
        }
        lastAssigned++;
        if (lastAssigned == meshAvailbyMPI.end()) lastAssigned = meshAvailbyMPI.begin();
      } while (lastAssigned != startedAt);
      if (mpiInstanceMap[i] == comm->id()) addToInNode(i);
    }
  }
}

int InNodeLargestQueueFirst::policyCheck(
    const std::map<int, gvt::render::actor::RayVector> &_queue) {
  std::shared_ptr<gvt::comm::communicator> comm = gvt::comm::communicator::singleton();
  int id = -1;
  std::size_t _total = 0;
  for (const auto &q : _queue) {
    if (mpiInstanceMap[q.first] == comm->id() && q.second.size() > _total) {
      id = q.first;
      _total = q.second.size();
    }
  }
  return id;
}

bool DomainTracer::areWeDone() {
  std::shared_ptr<gvt::comm::communicator> comm = gvt::comm::communicator::singleton();
  gvt::render::RenderContext &cntxt = *gvt::render::RenderContext::instance();
  std::shared_ptr<DomainTracer> tracer =
      std::dynamic_pointer_cast<DomainTracer>(cntxt.tracer());
  if (!tracer) return false;
  bool ret = tracer->_queue.empty();
  // std::cout << "[" << comm->id() << "] Check " << ((tracer->_queue.empty()) ? "T" :
  // "F")
  //           << std::endl << std::flush;
  return ret;
}

void DomainTracer::Done(bool T) {
  std::shared_ptr<gvt::comm::communicator> comm = gvt::comm::communicator::singleton();
  gvt::render::RenderContext &cntxt = *gvt::render::RenderContext::instance();
  std::shared_ptr<DomainTracer> tracer =
      std::dynamic_pointer_cast<DomainTracer>(cntxt.tracer());
  if (!tracer) return;
  // if (T)
  //   std::cout << "[" << comm->id() << "] Done ... " << (T ? "T" : "F") << std::endl
  //             << std::flush;
  tracer->GlobalFrameFinished = T;
}

DomainTracer::DomainTracer() : RayTracer() {
  RegisterMessage<gvt::comm::EmptyMessage>();
  RegisterMessage<gvt::comm::SendRayList>();
  std::shared_ptr<gvt::comm::communicator> comm = gvt::comm::communicator::singleton();

  v = std::make_shared<comm::vote::vote>(DomainTracer::areWeDone, DomainTracer::Done);
  comm->setVote(v);
}

DomainTracer::~DomainTracer() {}

void DomainTracer::operator()() {

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

  _queue.setQueuePolicy<InNodeLargestQueueFirst>();

  _cam->AllocateCameraRays();
  _cam->generateRays();

  int ray_portion = _cam->rays.size() / comm->lastid();
  int rays_start = comm->id() * ray_portion;
  size_t rays_end =
      (comm->id() + 1) == comm->lastid()
          ? _cam->rays.size()
          : (comm->id() + 1) * ray_portion; // tack on any odd rays to last proc

  // gvt::render::actor::RayVector lrays;
  // lrays.assign(_cam->rays.begin() + rays_start, _cam->rays.begin() + rays_end);
  // _cam->rays.clear();

  processRayQueue(_cam->rays);

  InNodeLargestQueueFirst *pp = dynamic_cast<InNodeLargestQueueFirst *>(_queue.QP.get());
  std::vector<int> remote_instances;
  if (pp) {
    for (auto q : _queue._queue) {
      if (pp->mpiInstanceMap[q.first] != comm->id()) {
        remote_instances.push_back(q.first);
      }
    }
    for (auto id : remote_instances) _queue._queue.erase(id);
  }

  GlobalFrameFinished = false;
  {
    gvt::core::time::timer t(true, "Tracing");
    while (!GlobalFrameFinished) {
      int target = -1;
      gvt::render::actor::RayVector toprocess, moved_rays, send_rays;
      _queue.dequeue(target, toprocess);
      if (target != -1) {
        trace(target, meshRef[target], toprocess, moved_rays, instM[target],
              instMinv[target], instMinvN[target], lights);
        processRayQueue(moved_rays, target);
      } else if (!_queue.empty()) {
        for (auto id : remote_instances) {
          if (_queue.dequeue_send(id, send_rays)) {
            // TODO: Send rays
            // std::cout << "Send queue : " << id << " " << send_rays.size() << std::endl
            // <<
            // std::flush;
            std::shared_ptr<gvt::comm::Message> msg =
                std::make_shared<gvt::comm::SendRayList>(
                    comm->id(), pp->mpiInstanceMap[id], send_rays);

            comm->send(msg, mpiInstanceMap[id]);
          }
        }
      }
      if (_queue.empty()) {
        v->PorposeVoting();
      }
    }
    t.stop();
  }
  {
    gvt::core::time::timer c(true, "Composite");
    float *img_final = composite_buffer->composite();
  }
};

bool DomainTracer::MessageManager(std::shared_ptr<gvt::comm::Message> msg) {
  std::cout << "TAG : " << msg->tag() << std::endl << std::flush;
  return Tracer::MessageManager(msg);
}

void DomainTracer::updateGeometry() {}
}
}
