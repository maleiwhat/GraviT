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

/** MpiRenderer.cpp
 *  A simple GraviT application that loads some geometry and renders it.
 */

#include "apps/render/MpiRenderer.h"

#include "gvt/render/RenderContext.h"
#include "gvt/render/Types.h"
#include <vector>
#include <algorithm>
#include <set>
#include "gvt/core/mpi/Wrapper.h"
#include "gvt/core/Math.h"
#include "gvt/render/data/Dataset.h"
#include "gvt/render/data/Domains.h"
#include "gvt/render/Schedulers.h"

#ifdef GVT_RENDER_ADAPTER_EMBREE
#include "gvt/render/adapter/embree/Wrapper.h"
#endif

#ifdef GVT_RENDER_ADAPTER_MANTA
#include "gvt/render/adapter/manta/Wrapper.h"
#endif

#ifdef GVT_RENDER_ADAPTER_OPTIX
#include "gvt/render/adapter/optix/Wrapper.h"
#endif

#include "gvt/render/algorithm/Tracers.h"
#include "gvt/render/data/scene/gvtCamera.h"
#include "gvt/render/data/scene/Image.h"
#include "gvt/render/data/Primitives.h"
#include "gvt/render/data/domain/reader/ObjReader.h"

#include <boost/range/algorithm.hpp>
#include "gvt/core/mpi/Application.h"

#include "gvt/render/unit/RequestWork.h"
#include "gvt/render/unit/TileWork.h"
#include "gvt/render/unit/PixelWork.h"

#include <iostream>
#include <mpi.h>

#define DEBUG_MPI_RENDERER

using namespace std;
using namespace gvt::render;
using namespace gvt::core::math;
using namespace gvt::core::mpi;
using namespace gvt::render::data::scene;
using namespace gvt::render::schedule;
using namespace gvt::render::data::primitives;
using namespace apps::render;

MpiRenderer::MpiRenderer(int *argc, char ***argv) :
  Application(argc, argv),
  camera(NULL), image(NULL), dbOption(NULL),
  tileLoadBalancer(NULL) {

  renderContext = gvt::render::RenderContext::instance();

  if (renderContext == NULL) {
    std::cout << "Something went wrong initializing the context" << std::endl;
    exit(0);
  }
}

MpiRenderer::~MpiRenderer() {
  if (camera != NULL) delete camera;
  if (image != NULL) delete image;
  if (dbOption != NULL) delete dbOption;
  if (tileLoadBalancer != NULL) delete tileLoadBalancer;
}

void MpiRenderer::parseCommandLine(int argc, char** argv) {

  this->dbOption = new TestDatabaseOption;

  if (argc > 1) {
    TestDatabaseOption* option = static_cast<TestDatabaseOption*>(dbOption);
    if (*argv[1] == 'i') {
      option->schedulerType =  gvt::render::scheduler::Image;
    } else if (*argv[1] == 'd') {
      option->schedulerType =  gvt::render::scheduler::Domain;
    }
    if (argc > 4) {
      option->instanceCountX = atoi(argv[2]);
      option->instanceCountY = atoi(argv[3]);
      option->instanceCountZ = atoi(argv[4]);
    }
  }  
}

void MpiRenderer::createDatabase() {
  // create data node
  Uuid dataNodeId = createNode("Data", "Data");

  std::string objName("bunny"); // TODO: fixed for now

  // add a mesh
  Uuid meshNodeId
      = addMesh(dataNodeId, objName + "_mesh",
                "../data/geom/" + objName + ".obj");

  // create instances node
  Uuid instancesNodeId = createNode("Instances", "Instances");

  // add instances
  Box3D meshBounds = getMeshBounds(meshNodeId);
  Vector3f extent = meshBounds.extent();

  const float gapX = extent[0] * 0.2f;
  const float gapY = extent[1] * 0.2f;
  const float gapZ = extent[2] * 0.2f;

  TestDatabaseOption* option = static_cast<TestDatabaseOption*>(dbOption);
  int instanceCountX = option->instanceCountX;
  int instanceCountY = option->instanceCountY;
  int instanceCountZ = option->instanceCountZ;

  Vector3f minPos((extent[0] + gapX) * instanceCountX * -0.5f,
                  (extent[1] + gapY) * instanceCountY * -0.5f,
                  (extent[2] + gapZ) * instanceCountZ * -0.5f);

  int instanceId = 0;
  for (int z=0; z<instanceCountZ; ++z) {
    for (int y=0; y<instanceCountY; ++y) {
      for (int x=0; x<instanceCountX; ++x) {
        // int instanceId = y * 2 + x;
        // m: transform matrix
        auto m = new gvt::core::math::AffineTransformMatrix<float>(true);
        *m = *m * gvt::core::math::AffineTransformMatrix<float>::
            createTranslation(minPos[0] + x * (extent[0] + gapX),
                              minPos[1] + y * (extent[1] + gapY),
                              minPos[2] + z * (extent[2] + gapZ));
        *m = *m * gvt::core::math::AffineTransformMatrix<float>::
            createScale(1.0f, 1.0f, 1.0f);
        Uuid instanceUuid =
            addInstance(instancesNodeId, meshNodeId,
                        instanceId++, objName, m);
      }
    }
  }

  // create lights node
  Uuid lightsNodeId = createNode("Lights", "Lights");

  // add lights
  Vector4f lightPosition(0.0, 0.1, 0.5, 0.0);
  Vector4f lightColor(1.0, 1.0, 1.0, 0.0);
  Uuid lightNodeId = addPointLight(lightsNodeId, "point_light",
                                   lightPosition, lightColor);

  // create camera node
  Point4f eye(0.0, 0.5, 1.2, 1.0);
  Point4f focus(0.0, 0.0, 0.0, 1.0);
  Vector4f upVector(0.0, 1.0, 0.0, 0.0);
  float fov = (45.0 * M_PI / 180.0);
  // const unsigned int width = 1920;
  // const unsigned int height = 1080;
  const unsigned int width = 640;
  const unsigned int height = 480;
  Uuid cameraNodeId =
      createCameraNode(eye, focus, upVector, fov, width, height);

  // create film node
  Uuid filmNodeId = createFilmNode(width, height, objName);

  // create the scheduler node
  // valid schedulers = {Image, Domain}
  // valid adapters = {Embree, Manta, Optix}
#ifdef GVT_RENDER_ADAPTER_EMBREE
  int adapterType = gvt::render::adapter::Embree;
#elif GVT_RENDER_ADAPTER_MANTA
  int adapterType = gvt::render::adapter::Manta;
#elif GVT_RENDER_ADAPTER_OPTIX
  int adapterType = gvt::render::adapter::Optix;
#else
  GVT_DEBUG(DBG_ALWAYS, "ERROR: missing valid adapter");
#endif

  adapterType = gvt::render::adapter::Embree;
  Uuid scheduleNodeId =
      createScheduleNode(option->schedulerType, option->adapterType);
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
  // add mesh to the database
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

  #ifdef DEBUG_MPI_RENDERER
  printf("[new instance %d] bounds: min(%.3f %.3f %.3f), max(%.3f %.3f %.3f)\n",
          instanceId, il[0], il[1], il[2], ih[0], ih[1], ih[2]);
  #endif

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

  // image = new Image(width, height, sceneName);

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

#define SERVER_CLIENT_MODEL

void MpiRenderer::render() {

  RequestWork::Register();
  TileWork::Register();
  PixelWork::Register();

  Start();

#ifdef SERVER_CLIENT_MODEL

  #ifdef DEBUG_MPI_RENDERER
  printf("Rank %d: world size: %d\n", GetRank(), GetSize());
  #endif

  int rank = GetRank();

  if (rank == rank::Server) {
    initServer();
  } else if (rank == rank::Display) {
    initDisplay();
  }

  MPI_Barrier(MPI_COMM_WORLD);
  
  if (rank != rank::Server && rank != rank::Display) { // workers
    initWorker();
  }

  Wait();

#else

  camera->AllocateCameraRays();
  camera->generateRays();

  gvt::core::DBNodeH root = renderContext->getRootNode();
  int width = gvt::core::variant_toInteger(root["Film"]["width"].value());
  int height = gvt::core::variant_toInteger(root["Film"]["height"].value());

  image = new Image(width, height, "image");

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

  QuitApplication(); 

#endif

}

void MpiRenderer::initServer() {

  DBNodeH root = renderContext->getRootNode();
  int width = gvt::core::variant_toInteger(root["Film"]["width"].value());
  int height = gvt::core::variant_toInteger(root["Film"]["height"].value());

  // TODO: hpark: For now, equally divide the image
  // try finer granularity later
  int numRanks = GetSize();
  int granularity = numRanks - 1;

  tileLoadBalancer = new TileLoadBalancer(width, height, granularity);
}

void MpiRenderer::initDisplay() {
  DBNodeH root = renderContext->getRootNode();
  int width = variant_toInteger(root["Film"]["width"].value());
  int height = variant_toInteger(root["Film"]["height"].value());
  image = new Image(width, height, "image");
  pendingPixelCount = width * height;
}

void MpiRenderer::initWorker() {
  DBNodeH root = renderContext->getRootNode();
  int width = variant_toInteger(root["Film"]["width"].value());
  int height = variant_toInteger(root["Film"]["height"].value());
  framebuffer.resize(width * height);
  RequestWork request;
  request.setSourceRank(GetRank());
  request.Send(rank::Server);
}
