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
/**
 * A simple GraviT application that loads some geometry and renders it.
 */

#include <apps/render/MpiRenderer.h>

#include <gvt/render/RenderContext.h>
#include <gvt/render/Types.h>
#include <vector>
#include <algorithm>
#include <set>
#include <gvt/core/mpi/Wrapper.h>
#include <gvt/core/Math.h>
#include <gvt/render/data/Dataset.h>
#include <gvt/render/data/Domains.h>
#include <gvt/render/Schedulers.h>

#ifdef GVT_RENDER_ADAPTER_EMBREE
#include <gvt/render/adapter/embree/Wrapper.h>
#endif

#ifdef GVT_RENDER_ADAPTER_MANTA
#include <gvt/render/adapter/manta/Wrapper.h>
#endif

#ifdef GVT_RENDER_ADAPTER_OPTIX
#include <gvt/render/adapter/optix/Wrapper.h>
#endif

#include <gvt/render/algorithm/Tracers.h>
#include <gvt/render/data/scene/gvtCamera.h>
#include <gvt/render/data/scene/Image.h>
#include <gvt/render/data/Primitives.h>
#include <gvt/render/data/domain/reader/ObjReader.h>

#include <boost/range/algorithm.hpp>
#include <gvt/core/mpi/Application.h>
#include <gvt/core/mpi/RenderWork.h>

#include <iostream>

using namespace std;
using namespace gvt::render;
using namespace gvt::core::math;
using namespace gvt::core::mpi;
using namespace gvt::render::data::scene;
using namespace gvt::render::schedule;
using namespace gvt::render::data::primitives;
using namespace apps::render;

MpiRenderer::MpiRenderer(int *argc, char ***argv) :
  camera(NULL), image(NULL) {
  // Application(argc, argv), camera(NULL), image(NULL) {

  renderContext = gvt::render::RenderContext::instance();

  if (renderContext == NULL) {
    std::cout << "Something went wrong initializing the context" << std::endl;
    exit(0);
  }
}

MpiRenderer::~MpiRenderer() {
  if (camera != NULL) delete camera;
  if (image != NULL) delete image;
}

bool MpiRenderer::isNodeTypeReserved(const std::string& type) {
  return ((type == std::string("Camera")) ||
          (type == std::string("Film")) ||
          (type == std::string("View")) ||
          (type == std::string("Dataset")) ||
          (type == std::string("Attribute")) ||
          (type == std::string("Mesh")) ||
          (type == std::string("Instance")) ||
          (type == std::string("PointLight")) ||
          (type == std::string("Schedule")));
}

DBNodeH MpiRenderer::getNode(const Uuid& id) {
  return renderContext->getNode(id); 
}

Uuid MpiRenderer::createNode(const std::string& type,
                             const std::string& name) {
  if (isNodeTypeReserved(type)) {
    std::cout << "error: reserved node type " << type << std::endl;
    exit(1);
  }
  gvt::core::DBNodeH root = renderContext->getRootNode();
  gvt::core::DBNodeH node =
      renderContext->createNodeFromType(type, name, root.UUID());
  return node.UUID();
}


Uuid MpiRenderer::addMesh(const Uuid& parentNodeId,
                          const std::string& meshName,
                          const std::string& objFilename) {
  gvt::core::DBNodeH node =
      renderContext->createNodeFromType("Mesh", meshName, parentNodeId);
  gvt::render::data::domain::reader::ObjReader objReader(objFilename);
  Mesh* mesh = objReader.getMesh();
  mesh->generateNormals();
  mesh->computeBoundingBox();
  Box3D* meshbbox = mesh->getBoundingBox();
  // add bunny mesh to the database
  node["file"] = objFilename;
  node["bbox"] = meshbbox;
  node["ptr"] = mesh;
  return node.UUID();
}

Box3D MpiRenderer::getMeshBounds(const Uuid& id) {
  gvt::core::DBNodeH meshNode = renderContext->getNode(id);
  Box3D* bounds = gvt::core::variant_toBox3DPtr(meshNode["bbox"].value());
  return *bounds;
}

Uuid MpiRenderer::
    addInstance(const Uuid& parentNodeId,
                const Uuid& meshId,
                int instanceId,
                const std::string& instanceName,
                gvt::core::math::AffineTransformMatrix<float>* transform) {
  gvt::core::DBNodeH node =
      renderContext->createNodeFromType("Instance", instanceName,
                                        parentNodeId);

  gvt::core::DBNodeH meshNode = renderContext->getNode(meshId);
  Box3D* mbox = gvt::core::variant_toBox3DPtr(meshNode["bbox"].value());

  node["id"] = instanceId; // unique id per instance
  node["meshRef"] = meshNode.UUID();

  // transform the instance
  // auto m = new gvt::core::math::AffineTransformMatrix<float>(true);
  auto minv = new gvt::core::math::AffineTransformMatrix<float>(true);
  auto normi = new gvt::core::math::Matrix3f();

  auto m = transform;
  node["mat"] = m;
  *minv = m->inverse();
  node["matInv"] = minv;
  *normi = m->upper33().inverse().transpose();
  node["normi"] = normi;

  // transform mesh bounding box
  auto il = (*m) * mbox->bounds[0];
  auto ih = (*m) * mbox->bounds[1];
  Box3D *ibox = new gvt::render::data::primitives::Box3D(il, ih);
  node["bbox"] = ibox;
  node["centroid"] = ibox->centroid();

  return node.UUID();
}

Uuid MpiRenderer::addPointLight(const Uuid& parentNodeId,
                                const std::string& lightName,
                                const Vector4f& position,
                                const Vector4f& color) {
  gvt::core::DBNodeH node =
      renderContext->createNodeFromType("PointLight", lightName, parentNodeId);
  node["position"] = Vector4f(0.0, 0.1, 0.5, 0.0);
  node["color"] = Vector4f(1.0, 1.0, 1.0, 0.0);
  return node.UUID();
}

Uuid MpiRenderer::createCameraNode(const Point4f& eye,
                                   const Point4f& focus,
                                   const Vector4f& upVector,
                                   float fov,
                                   unsigned int width, 
                                   unsigned int height) {

  gvt::core::DBNodeH root = renderContext->getRootNode();

  gvt::core::DBNodeH node =
      renderContext->createNodeFromType("Camera", "cam", root.UUID());

  node["eyePoint"] = eye;
  node["focus"] = focus;
  node["upVector"] = upVector;
  node["fov"] = fov;

  camera = new gvtPerspectiveCamera();
  camera->lookAt(eye, focus, upVector);
  camera->setFOV(fov);
  camera->setFilmsize(width, height);

  return node.UUID();
}

Uuid MpiRenderer::createFilmNode(int width,
                                 int height,
                                 const std::string& sceneName) {

  gvt::core::DBNodeH root = renderContext->getRootNode();

  gvt::core::DBNodeH node =
      renderContext->createNodeFromType("Film", "film", root.UUID());

  node["width"] = width;
  node["height"] = height;

  image = new Image(width, height, sceneName);

  return node.UUID();
}

Uuid MpiRenderer::createScheduleNode(int schedulerType, int adapterType) {

  gvt::core::DBNodeH root = renderContext->getRootNode();

  gvt::core::DBNodeH node =
      renderContext->createNodeFromType("Schedule", "sched", root.UUID());

  node["type"] = schedulerType;
  node["adapter"] = adapterType;

  return node.UUID();
}

void MpiRenderer::render() {

  camera->AllocateCameraRays();
  camera->generateRays();

  gvt::core::DBNodeH root = renderContext->getRootNode();

  int schedType =
      gvt::core::variant_toInteger(root["Schedule"]["type"].value());
  switch (schedType) {
    case gvt::render::scheduler::Image: {
      std::cout << "starting image scheduler" << std::endl;
      gvt::render::algorithm::Tracer<ImageScheduler>(camera->rays, *image)();
      break;
    }
    case gvt::render::scheduler::Domain: {
      std::cout << "starting domain scheduler" << std::endl;
      gvt::render::algorithm::Tracer<DomainScheduler>(camera->rays, *image)();
      break;
    }
    default: {
      std::cout << "unknown schedule type provided: " << schedType << std::endl;
      break;
    }
  }
  std::cout << "writing image" << std::endl; 
  image->Write();
}

