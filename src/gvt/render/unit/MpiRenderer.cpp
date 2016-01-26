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

#include "gvt/render/unit/MpiRenderer.h"

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
#include "gvt/render/data/accel/BVH.h"

#include <boost/range/algorithm.hpp>
#include "gvt/core/mpi/Application.h"

#include "gvt/render/unit/RequestWork.h"
#include "gvt/render/unit/TileWork.h"
#include "gvt/render/unit/ImageTileWork.h"
#include "gvt/render/unit/DomainTileWork.h"
#include "gvt/render/unit/PixelWork.h"
#include "gvt/render/unit/RayWork.h"
#include "gvt/render/unit/DoneTestWork.h"
#include "gvt/render/unit/PixelGatherWork.h"
#include "gvt/render/unit/TileLoadBalancer.h"

#include <iostream>
#include <mpi.h>

#define DEBUG_MPI_RENDERER
// #define SEPARATE_SERVER_WORKERS

using namespace std;
using namespace gvt::render;
using namespace gvt::core;
using namespace gvt::core::math;
using namespace gvt::core::mpi;
using namespace gvt::render::data::scene;
using namespace gvt::render::schedule;
using namespace gvt::render::data::primitives;
using namespace gvt::render::unit;

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

  // TODO (hpark): generalize this
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
  // Uuid meshNodeId
  //     = addMesh(dataNodeId, objName + "_mesh",
  //               "../data/geom/" + objName + ".obj");

  // create instances node
  Uuid instancesNodeId = createNode("Instances", "Instances");

  // add instances
  // Box3D meshBounds = getMeshBounds(meshNodeId);
  Box3D meshBounds = getMeshBounds("../data/geom/" + objName + ".obj");
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
        // add a mesh
        Uuid meshNodeId
            = addMesh(dataNodeId, objName + "_mesh",
                      "../data/geom/" + objName + ".obj");

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
  node["bbox"] = (unsigned long long)meshbbox;
  node["ptr"] = (unsigned long long)mesh;
  return node.UUID();
}

Box3D MpiRenderer::getMeshBounds(const std::string& objFilename) {
  gvt::render::data::domain::reader::ObjReader objReader(objFilename);
  Mesh* mesh = objReader.getMesh();
  // mesh->generateNormals();
  mesh->computeBoundingBox();
  Box3D* bounds = mesh->getBoundingBox();
  return *bounds;
}

Box3D MpiRenderer::getMeshBounds(const Uuid& id) {
  gvt::core::DBNodeH meshNode = renderContext->getNode(id);
  // Box3D* bounds = *gvt::core::variant_toBox3DPtr(meshNode["bbox"].value());
  Box3D* bounds = (Box3D*)meshNode["bbox"].value().toULongLong();
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
  // Box3D* mbox = *gvt::core::variant_toBox3DPtr(meshNode["bbox"].value());
  Box3D* mbox = (Box3D*)meshNode["bbox"].value().toULongLong();

  node["id"] = instanceId; // unique id per instance
  node["meshRef"] = meshNode.UUID();

  // transform the instance
  // auto m = new gvt::core::math::AffineTransformMatrix<float>(true);
  auto minv = new gvt::core::math::AffineTransformMatrix<float>(true);
  auto normi = new gvt::core::math::Matrix3f();

  auto m = transform;
  node["mat"] = (unsigned long long)m;
  *minv = m->inverse();
  node["matInv"] = (unsigned long long)minv;
  *normi = m->upper33().inverse().transpose();
  node["normi"] = (unsigned long long)normi;

  // transform mesh bounding box
  auto il = (*m) * mbox->bounds[0];
  auto ih = (*m) * mbox->bounds[1];
  Box3D *ibox = new gvt::render::data::primitives::Box3D(il, ih);
  node["bbox"] = (unsigned long long)ibox;
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

void MpiRenderer::initInstanceRankMap() {

  gvt::core::Vector<gvt::core::DBNodeH> dataNodes = root["Data"].getChildren();

  std::cout<<"instance node size: "<<instanceNodes.size()
           <<"data node size: "<<dataNodes.size()<<"\n";

  // create a map of instances to mpi rank
  for (size_t i = 0; i < instanceNodes.size(); ++i) {
    gvt::core::DBNodeH meshNode = instanceNodes[i]["meshRef"].deRef();
    size_t dataIdx = -1;
    for (size_t d = 0; d < dataNodes.size(); ++d) {
      if (dataNodes[d].UUID() == meshNode.UUID()) {
        dataIdx = d;
        break;
      }
    }
    // NOTE: mpi-data(domain) assignment strategy
    int ownerRank = static_cast<int>(dataIdx) % GetSize();
    GVT_DEBUG(DBG_ALWAYS, "[" << GetRank() << "] domain scheduler: instId: "
                              << i << ", dataIdx: " << dataIdx
                              << ", target mpi node: " << ownerRank
                              << ", world size: " << GetSize());
    std::cout<<"[" << GetRank() << "] domain scheduler: instId: "
                              << i << ", dataIdx: " << dataIdx
                              << ", target mpi node: " << ownerRank
                              << ", world size: " << GetSize() << "\n";
    GVT_ASSERT(dataIdx != (size_t)-1,
               "domain scheduler: could not find data node");
    instanceRankMap[instanceNodes[i].UUID()] = ownerRank;
  }
}

void MpiRenderer::setupRender() {
  root = renderContext->getRootNode();
  instanceNodes = root["Instances"].getChildren();
  GVT_DEBUG(DBG_ALWAYS, "num instances: " << instanceNodes.size());
  imageWidth = root["Film"]["width"].value().toInteger();
  imageHeight = root["Film"]["height"].value().toInteger();
  framebuffer.resize(imageWidth * imageHeight);
  acceleration = new gvt::render::data::accel::BVH(instanceNodes);
  rayQueueMutex = new tbb::mutex[instanceNodes.size()];
  colorBufMutex = new tbb::mutex[imageWidth];

  int schedType = root["Schedule"]["type"].value().toInteger();
  if (schedType == scheduler::Domain) {
    initInstanceRankMap();
    doneTestRunning = false;
    allWorkDone = false;
    pthread_mutex_init(&doneTestLock, NULL);
    pthread_cond_init(&doneTestCondition, NULL);
  }
}

void MpiRenderer::freeRender() {
  delete acceleration;
  delete [] rayQueueMutex;
  delete [] colorBufMutex;
}

void MpiRenderer::render() {

  setupRender();
  int schedType = root["Schedule"]["type"].value().toInteger();
  // TODO (hpark):
  // collapse the following two if-else blocks into a single block
  if (schedType == scheduler::Image) {
  
    RequestWork::Register();
    TileWork::Register();
    ImageTileWork::Register();
    PixelWork::Register();
  
    Start();
  
    #ifdef DEBUG_MPI_RENDERER
    printf("Rank %d: world size: %d\n", GetRank(), GetSize());
    #endif
  
    int rank = GetRank();
  
    if (rank == rank::Server) {
      initServer();
    } else {
      initDisplay();
    }
  
    MPI_Barrier(MPI_COMM_WORLD);
    
    if (rank != rank::Server && rank != rank::Display) { // workers
      initWorker();
    }
  
    Wait();
  
    freeRender();

  } else {
  
    // setupRender();
  
    DomainTileWork::Register();
    RayWork::Register();
    DoneTestWork::Register();
    PixelGatherWork::Register();
  
    Start();
  
    image = new Image(imageWidth, imageHeight, "image");
    DomainTileWork work;
    work.setTileSize(0, 0, imageWidth, imageHeight);
    work.Action();
  
    Wait();
    freeRender();
  }
}

void MpiRenderer::initServer() {
  // TODO (hpark): For now, equally divide the image
  // try coarser/finer granularity later
  int schedType = root["Schedule"]["type"].value().toInteger();
  int numRanks = GetSize();
  int numWorkers = numRanks - rank::FirstWorker;
  int granularity = numWorkers;

  tileLoadBalancer =
      new TileLoadBalancer(schedType, imageWidth, imageHeight,
                           granularity, numWorkers);
}

void MpiRenderer::initDisplay() {
  image = new Image(imageWidth, imageHeight, "image");
  pendingPixelCount = imageWidth * imageHeight;
}

void MpiRenderer::initWorker() {
  // framebuffer.resize(imageWidth * imageHeight);
  RequestWork request;
  request.setSourceRank(GetRank());
  request.Send(rank::Server);
}

void MpiRenderer::aggregatePixel(int pixelId, const GVT_COLOR_ACCUM& addend) {
  GVT_COLOR_ACCUM& color = framebuffer[pixelId];
  color.add(addend);
  color.rgba[3] = 1.f;
  color.clamp();
}
