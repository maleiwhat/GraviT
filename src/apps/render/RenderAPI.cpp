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

   GraviT is funded in part by the US National Science Foundation under awards ACI-1339863,
   ACI-1339881 and ACI-1339840
   ======================================================================================= */
/**
 * Provides a simple API for external applications to easily feed data and render images.
 *
*/


#include "RenderAPI.h"

#include <iostream>
#include <algorithm>
#include <gvt/core/Math.h>
#include <gvt/core/context/Variant.h>
#include <gvt/render/RenderContext.h>
#include <gvt/render/Schedulers.h>
#include <gvt/render/Types.h>
#include <gvt/render/data/Domains.h>
#include <set>
#include <mpi.h>

#include "gvt/core/Types.h"
#include <limits>

#include <tbb/task_scheduler_init.h>
#include <thread>

#ifdef GVT_RENDER_ADAPTER_EMBREE
#include <gvt/render/adapter/embree/EmbreeMeshAdapter.h>
#endif

#ifdef GVT_RENDER_ADAPTER_MANTA
#include <gvt/render/adapter/manta/MantaMeshAdapter.h>
#endif

#ifdef GVT_RENDER_ADAPTER_OPTIX
#include <gvt/render/adapter/optix/OptixMeshAdapter.h>
#endif

#include <gvt/render/algorithm/Tracers.h>
#include <gvt/render/data/Primitives.h>
#include <gvt/render/data/scene/Image.h>
#include <gvt/render/data/scene/gvtCamera.h>
#include <mpi.h>
#include <float.h>

using namespace std;
using namespace gvt::render;

using namespace gvt::render::data::scene;
using namespace gvt::render::schedule;
using namespace gvt::render::data::primitives;

void VisitAdapter::initMPI() {
  int mpiInitlized = 0;
  MPI_Initialized(&mpiInitlized);
  if (!mpiInitlized) {
    GVT_DEBUG(DBG_ALWAYS, "Gravit Adapter MPI Init not called, calling MPI Init with null args.");

    int dummyArgc = 1;
    char ** argv = NULL;
    MPI_Init(&dummyArgc, &argv);
  }
}

void VisitAdapter::setRayTraceProperties(RayTraceProperties properties) {
  gravitProgramConfig.maxDepth = properties.maxDepth;
  gravitProgramConfig.raySamples = properties.raySamples;
  gravitProgramConfig.windowJitterSize = properties.windowJitterSize;
  gravitProgramConfig.backgroundColor[0] = properties.backgroundColor[0];
  gravitProgramConfig.backgroundColor[1] = properties.backgroundColor[1];
  gravitProgramConfig.backgroundColor[2] = properties.backgroundColor[2];
}

void VisitAdapter::setLight(int numberOfLights, LightType * lighttypes, double * lightDirection, unsigned char * color,
                            double * lightIntensity) {
  gvt::render::RenderContext * cntxt = gvt::render::RenderContext::instance();
  if (cntxt == NULL) {
    std::cout << "Something went wrong initializing the context" << std::endl;
    exit(0);
  }

  gvt::core::DBNodeH root = cntxt->getRootNode();

  // check if lightnodes has already being created

  gvt::core::DBNodeH lightNodes = cntxt->findChildNodeByName("Lights", root.UUID());
  if (!lightNodes) {
    lightNodes = cntxt->createNodeFromType("Lights", "Lights", root.UUID());
  } else {
    cntxt->deleteChildren(lightNodes.UUID());
  }

  for (int i = 0; i < numberOfLights; i++) {
    float distanceMod = -1;

    LightType lType = *(lighttypes + i);

    double * center;
    double dataCenter[3] = { -1, -1, -1};

    // calculate data diag size
    float a = powf(gravitProgramConfig.dataBoundingBox[0] - gravitProgramConfig.dataBoundingBox[4], 2);
    float b = powf(gravitProgramConfig.dataBoundingBox[1] - gravitProgramConfig.dataBoundingBox[5], 2);
    float c = powf(gravitProgramConfig.dataBoundingBox[2] - gravitProgramConfig.dataBoundingBox[6], 2);

    float diag = powf(a + b + c, 0.5);

    if (lType == camera) { // if it is camera, focus around center, else focus around data center
      center = gravitProgramConfig.focalPoint;
    } else {
      dataCenter[0] = (gravitProgramConfig.dataBoundingBox[0] - gravitProgramConfig.dataBoundingBox[4]) / 2.0;
      dataCenter[1] = (gravitProgramConfig.dataBoundingBox[1] - gravitProgramConfig.dataBoundingBox[5]) / 2.0;
      dataCenter[2] = (gravitProgramConfig.dataBoundingBox[2] - gravitProgramConfig.dataBoundingBox[6]) / 2.0;
      center = dataCenter;
    }

    distanceMod = diag;

    double * lDirection = lightDirection + i * 3;
    unsigned  char * lcolor = color + i * 3;
    double intensity = *(lightIntensity + i);

    glm::vec3 vLdirection(lDirection[0], lDirection[1], lDirection[2]);
    vLdirection = -vLdirection;

    glm::vec3 lPosition = glm::vec3(center[0], center[1], center[2]) + vLdirection * float(distanceMod * 1.3 / intensity);

    if (lType == camera) { // this is camera
      //tranform camera space light into world space light

      glm::vec3 v(gravitProgramConfig.upVector[0], gravitProgramConfig.upVector[1], gravitProgramConfig.upVector[2]); //up, should be the true up
      glm::vec3 n(gravitProgramConfig.view_direction[0], gravitProgramConfig.view_direction[1], gravitProgramConfig.view_direction[2]); //foward
      v = -v;
      n = -n;
      glm::vec3 u = glm::cross(v, n);


      glm::mat3 basis(u, v, n);

      lPosition = basis * lPosition;
    }

    // assume for now that everthing is a point light
    std::string baseName = "conelight";
    baseName += baseName + std::to_string(i);

    gvt::core::DBNodeH lightNode = cntxt->createNodeFromType("PointLight", baseName.c_str(), lightNodes.UUID());
    gravitProgramConfig.lightWorldCoordinates.push_back(lPosition[0]);
    gravitProgramConfig.lightWorldCoordinates.push_back(lPosition[1]);
    gravitProgramConfig.lightWorldCoordinates.push_back(lPosition[2]);
     
    lightNode["position"] = lPosition;

    lightNode["color"] = glm::vec3((unsigned int)lcolor[0] / (float)255, (unsigned int)lcolor[1] / (float)255, (unsigned int)lcolor[2] / (float)255);
  }
}

void VisitAdapter::changeMaterial(int meshId, int material, double * materialProp) {
  gvt::render::RenderContext * cntxt = gvt::render::RenderContext::instance();
  if (cntxt == NULL) {
    std::cout << "Something went wrong initializing the context" << std::endl;
    exit(0);
  }

  gvt::core::DBNodeH root = cntxt->getRootNode();

  gvt::core::DBNodeH instanceNode = cntxt->findChildNodeByName("Instances", root.UUID());
  if (!instanceNode) {
    return;
  }

  gvt::core::Vector<gvt::core::DBNodeH> instancenodes = root["Instances"].getChildren();
  std::map<int, gvt::render::data::primitives::Mesh *> meshRef;

  for (int i = 0; i < instancenodes.size(); i++) {

    meshRef[i] = (gvt::render::data::primitives::Mesh *)instancenodes[i]["meshRef"].deRef()["ptr"].value().toULongLong();

    gvt::render::data::primitives::Mesh * mesh = meshRef[i];

    gvt::render::data::primitives::Material * mat;

    switch (material) {
      case 0:
        mat = new Material();
        mat->type = MATERIAL_TYPE::LAMBERT;
        mat->kd = glm::vec3(materialProp[0], materialProp[1], materialProp[2]);
        break;
      case 1:
        mat = new Material();
        mat->type = MATERIAL_TYPE::PHONG;
        mat->kd = glm::vec3(materialProp[0], materialProp[1], materialProp[2]);
        mat->ks = glm::vec3(materialProp[4], materialProp[5], materialProp[6]);
        break;
      case 2:
        mat = new Material();
        mat->type = MATERIAL_TYPE::BLINN;
        mat->kd = glm::vec3(materialProp[0], materialProp[1], materialProp[2]);
        mat->ks = glm::vec3(materialProp[4], materialProp[5], materialProp[6]);
        break;
    }
    mesh->setMaterial(mat);
  }
}

void VisitAdapter::resetMeshAndInstance() {
  gravitProgramConfig.dataBoundingBox[0] = std::numeric_limits<double>::max();
  gravitProgramConfig.dataBoundingBox[1] = std::numeric_limits<double>::max();
  gravitProgramConfig.dataBoundingBox[2] = std::numeric_limits<double>::max();

  gravitProgramConfig.dataBoundingBox[3] = std::numeric_limits<double>::min();
  gravitProgramConfig.dataBoundingBox[4] = std::numeric_limits<double>::min();
  gravitProgramConfig.dataBoundingBox[5] = std::numeric_limits<double>::min();

  gvt::render::RenderContext * cntxt = gvt::render::RenderContext::instance();
  if (cntxt == NULL) {
    std::cout << "Something went wrong initializing the context" << std::endl;
    exit(0);
  }
  gvt::core::DBNodeH root = cntxt->getRootNode();

  gvt::core::DBNodeH dataNodes = cntxt->findChildNodeByName("Data", root.UUID());
  if (dataNodes) {
    cntxt->deleteChildren(dataNodes.UUID());
  }

  gvt::core::DBNodeH instNodes = cntxt->findChildNodeByName("Instances", root.UUID());
  if (instNodes) {
    cntxt->deleteChildren(instNodes.UUID());
  }
}

void VisitAdapter::setCamera(int * imageSize, double * focalPoint, double * upVector,
                             double * viewDirection, double zoom, double fov) {
  gravitProgramConfig.focalPoint[0] = focalPoint[0];
  gravitProgramConfig.focalPoint[1] = focalPoint[1];
  gravitProgramConfig.focalPoint[2] = focalPoint[2];

  gravitProgramConfig.upVector[0] = -upVector[0];
  gravitProgramConfig.upVector[1] = -upVector[1];
  gravitProgramConfig.upVector[2] = -upVector[2];

  gravitProgramConfig.fov = fov;
  gravitProgramConfig.zoom = zoom;

  // need to invert view normal
  gravitProgramConfig.view_direction[0] = -viewDirection[0];
  gravitProgramConfig.view_direction[1] = -viewDirection[1];
  gravitProgramConfig.view_direction[2] = -viewDirection[2];

  gravitProgramConfig.filmSize[0] = imageSize[0];
  gravitProgramConfig.filmSize[1] = imageSize[1];

  // calculate cameraposition from direction and zoom
  glm::vec3 glmfocalPoint = glm::vec3(gravitProgramConfig.focalPoint[0], gravitProgramConfig.focalPoint[1], gravitProgramConfig.focalPoint[2]);
  glm::vec3 direction = glm::vec3(gravitProgramConfig.view_direction[0], gravitProgramConfig.view_direction[1], gravitProgramConfig.view_direction[2]);
  glm::vec3 camPosition = glmfocalPoint - (1.4f * float(gravitProgramConfig.zoom) * direction);

  gravitProgramConfig.cameraWorldCoordinates[0] = camPosition[0];
  gravitProgramConfig.cameraWorldCoordinates[1] = camPosition[1];
  gravitProgramConfig.cameraWorldCoordinates[2] = camPosition[2];
}

void VisitAdapter::draw(unsigned char * image) {
  gvt::render::RenderContext * cntxt = gvt::render::RenderContext::instance();
  if (cntxt == NULL) {
    std::cout << "Something went wrong initializing the context" << std::endl;
    exit(0);
  }

  gvt::core::DBNodeH root = cntxt->getRootNode();

  gvt::core::DBNodeH camNode = cntxt->createNodeFromType("Camera", "conecam", root.UUID());
  gvt::core::DBNodeH filmNode = cntxt->createNodeFromType("Film", "conefilm", root.UUID());

  filmNode["width"] = gravitProgramConfig.filmSize[0];
  filmNode["height"] = gravitProgramConfig.filmSize[1];

  gvtPerspectiveCamera mycamera;

  // calculate cameraposition from direction and zoom
  glm::vec3 focalPoint = glm::vec3(gravitProgramConfig.focalPoint[0],
                                   gravitProgramConfig.focalPoint[1],
                                   gravitProgramConfig.focalPoint[2]);

  glm::vec3 direction = glm::vec3(gravitProgramConfig.view_direction[0],
                                  gravitProgramConfig.view_direction[1],
                                  gravitProgramConfig.view_direction[2]);

  glm::vec3 camPosition = glm::vec3(gravitProgramConfig.cameraWorldCoordinates[0],
                                    gravitProgramConfig.cameraWorldCoordinates[1],
                                    gravitProgramConfig.cameraWorldCoordinates[2]);


  float fov = gravitProgramConfig.fov;
  glm::vec3 up = glm::vec3(gravitProgramConfig.upVector[0], gravitProgramConfig.upVector[1], gravitProgramConfig.upVector[2]);

  int rayMaxDepth = gravitProgramConfig.maxDepth;
  int raySamples = gravitProgramConfig.raySamples;
  float jitterWindowSize = gravitProgramConfig.windowJitterSize;

  mycamera.setMaxDepth(rayMaxDepth);
  mycamera.setSamples(raySamples);
  mycamera.setJitterWindowSize(jitterWindowSize);
  mycamera.lookAt(camPosition, focalPoint, up);
  mycamera.setFOV(fov);
  mycamera.setFilmsize(gravitProgramConfig.filmSize[0], gravitProgramConfig.filmSize[1]);

  gvt::core::DBNodeH schedNode = cntxt->createNodeFromType("Schedule", "Enzosched", root.UUID());


#ifdef GVT_RENDER_ADAPTER_EMBREE
  int adapterType = gvt::render::adapter::Embree;
#elif GVT_RENDER_ADAPTER_MANTA
  int adapterType = gvt::render::adapter::Manta;
#elif GVT_RENDER_ADAPTER_OPTIX
  int adapterType = gvt::render::adapter::Optix;
#else
  GVT_DEBUG(DBG_ALWAYS, "ERROR: missing valid adapter");
#endif
  schedNode["adapter"] = adapterType;

  // setup image from database sizes

  Image myimage(mycamera.getFilmSizeWidth(), mycamera.getFilmSizeHeight(), "image");

  mycamera.AllocateCameraRays();
  mycamera.generateRays();

  if (gravitProgramConfig.traceScheduler == Scheduler::image) {
    GVT_DEBUG(DBG_ALWAYS, "Rendering in image tracer mode");

    schedNode["type"] = gvt::render::scheduler::Image;

    gvt::render::algorithm::Tracer<ImageScheduler> tracer(mycamera.rays, myimage);
    tracer.sample_ratio = 1.0 / float(raySamples * raySamples);
    tracer.loadBlockFunc = loadBlockFunc;
    tracer.loadBlockObj = loadBlockObj;
    tracer();
  } else if (gravitProgramConfig.traceScheduler == Scheduler::domain) {
    GVT_DEBUG(DBG_ALWAYS, "Rendering in domain tracer mode");

    schedNode["type"] = gvt::render::scheduler::Domain;
    gvt::render::algorithm::Tracer<DomainScheduler> tracer(mycamera.rays, myimage);
    tracer.sample_ratio = 1.0 / float(raySamples * raySamples);
    tracer.loadBlockFunc = NULL;
    tracer.loadBlockObj = NULL;
    tracer();
  }

  if (MPI::COMM_WORLD.Get_rank() != 0) {
    return;
  }

  myimage.WriteChar(image);
}

void VisitAdapter::setBoundingBoxHolder(double * points, int numPoints, int * edges, int numEdges, int material, double * materialProp) {
  setData(points, numPoints, edges, numEdges, material, materialProp);
}

void VisitAdapter::setData(double * points, int numPoints, int * edges, int numEdges, int material, double * materialProp) {

  int rank = -1;
  MPI_Comm_rank(MPI_COMM_WORLD, &rank);

  gvt::render::RenderContext * cntxt = gvt::render::RenderContext::instance();
  if (cntxt == NULL) {
    std::cout << "Something went wrong initializing the context" << std::endl;
    exit(0);
  }

  gvt::core::DBNodeH root = cntxt->getRootNode();
  gvt::core::DBNodeH dataNodes = cntxt->findChildNodeByName("Data", root.UUID());

  root+= cntxt->createNode("threads", (int)std::thread::hardware_concurrency());

  if (!dataNodes && rank == 0) {
    dataNodes = cntxt->addToSync(cntxt->createNodeFromType("Data", "Data", root.UUID()));
    cntxt->addToSync(cntxt->createNodeFromType("Instances", "Instances", root.UUID()));
  }

  if (rank == 0) {
    gvt::core::DBNodeH boundingBoxes = cntxt->findChildNodeByName("BoundingBoxes", root.UUID());
    if (!boundingBoxes) {
      boundingBoxes = cntxt->createNodeFromType("BoundingBoxes", "boundBoxes", root.UUID());
      cntxt->addToSync(boundingBoxes);
      gvt::core::DBNodeH bbLower = cntxt->createNodeFromType("Lower", "lower", boundingBoxes.UUID());
      gvt::core::DBNodeH bbUpper = cntxt->createNodeFromType("Upper", "upper", boundingBoxes.UUID());
    }
  }

  cntxt->syncContext();

  dataNodes = root["Data"];

  gvt::core::DBNodeH meshNode;
  meshNode = cntxt->createNodeFromType("Mesh", "meshNode", dataNodes.UUID());
  cntxt->addToSync(meshNode);

  gvt::core::DBNodeH instNodes = root["Instances"];
  {
    Mesh * mesh;
    Material * mat;
    switch (material) {
      case 0:
        mat = new Material();
        mat->type = MATERIAL_TYPE::LAMBERT;
        mat->kd = glm::vec3(materialProp[0], materialProp[1], materialProp[2]);
        mesh = new Mesh(mat);
        break;
      case 1:
        mat = new Material();
        mat->type = MATERIAL_TYPE::PHONG;
        mat->kd = glm::vec3(materialProp[0], materialProp[1], materialProp[2]);
        mat->ks = glm::vec3(materialProp[4], materialProp[5], materialProp[6]);
        mesh = new Mesh(mat);
        break;
      case 2:
        mat = new Material();
        mat->type = MATERIAL_TYPE::BLINN;
        mat->kd = glm::vec3(materialProp[0], materialProp[1], materialProp[2]);
        mat->ks = glm::vec3(materialProp[4], materialProp[5], materialProp[6]);
        mesh = new Mesh(mat);
        break;
    }
    double * point = points;

    glm::vec3 lower(FLT_MAX, FLT_MAX, FLT_MAX);
    glm::vec3 upper(FLT_MIN, FLT_MIN, FLT_MIN);


    for (int i = 0; i < numPoints; i++) {
      glm::vec3 newPoint(*point, *(point + 1), *(point + 2));
      mesh->addVertex(newPoint);
      for (int j = 0; j < 3; j++) {
        lower[j] = (lower[j] < newPoint[j]) ? lower[j] : newPoint[j];
        upper[j] = (upper[j] > newPoint[j]) ? upper[j] : newPoint[j];
      }
      point += 3;
    }

    int * edge = edges;
    for (int i = 0; i < numEdges; i++) {
      mesh->addFace(*(edge), *(edge + 1), *(edge + 2));
      edge += 3;
    }
    //add mesh bounding box information

    gvt::core::DBNodeH boundingBoxes = root["BoundingBoxes"];
    gvt::core::DBNodeH lowerBoundingBoxes = boundingBoxes["Lower"];
    gvt::core::DBNodeH upperBoundingBoxes = boundingBoxes["Upper"];

    gvt::core::DBNodeH meshLower = cntxt->createNodeFromType("MeshLower", "meshLower", lowerBoundingBoxes.UUID());
    gvt::core::DBNodeH meshUpper = cntxt->createNodeFromType("MeshUpper", "meshUpper", upperBoundingBoxes.UUID());
    meshLower = lower;
    meshUpper = upper;

    mesh->generateNormals();

    // calculate bbox
    Box3D * meshbbox = new gvt::render::data::primitives::Box3D(lower, upper);

    // add cone mesh to the database
    gvt::core::Variant meshvariant(mesh);
    meshNode["file"] = string("/fake/path/to/cone");
    meshNode["bbox"] = (unsigned long long)meshbbox;
    meshNode["ptr"] = (unsigned long long)mesh;
    gvt::core::DBNodeH loc = cntxt->createNode("rank", rank);
    meshNode["Locations"] += loc;
    cntxt->addToSync(meshNode);
  }

  cntxt->syncContext();

  // calculate bounding box for entire object
  gvt::core::DBNodeH boundingBoxes = root["BoundingBoxes"];
  gvt::core::DBNodeH lowerBoundingBoxes = boundingBoxes["Lower"];
  gvt::core::DBNodeH upperBoundingBoxes = boundingBoxes["Upper"];

  int totalLowerBBs = lowerBoundingBoxes.getChildren().size();
  for (int i = 0; i < totalLowerBBs; i++ ) {
    //find global lower
    glm::vec3 lowerBB = lowerBoundingBoxes.getChildren()[i].value().tovec3();

    //find global upper
    glm::vec3 upperBB = upperBoundingBoxes.getChildren()[i].value().tovec3();

    for (int i = 0; i < 3; i++) {
      gravitProgramConfig.dataBoundingBox[i]  = min((float)gravitProgramConfig.dataBoundingBox[i], lowerBB[i]);
      gravitProgramConfig.dataBoundingBox[i + 3]  = min((float)gravitProgramConfig.dataBoundingBox[i + 3], upperBB[i]);
    }
  }

  gvt::core::DBNodeH instnode = cntxt->createNodeFromType("Instance", "inst", instNodes.UUID());

  Box3D * mbox = (Box3D *)meshNode["bbox"].value().toULongLong();
  int iIndex = instNodes.getChildren().size();
  instnode["id"] = iIndex - 1; // unique id per instance
  instnode["meshRef"] = meshNode.UUID();

  // transform data
  float scale = 1.0;
  auto m = new glm::mat4(1.f);
  auto minv = new glm::mat4(1.f);
  auto normi = new glm::mat3(1.f);
  //*m = glm::translate(*m, glm::vec3(0, 0, 0));
  //*m *glm::mat4::createTranslation(0.0, 0.0, 0.0);
  //*m = *m * glm::mat4::createScale(scale, scale, scale);
  *m = glm::scale(*m, glm::vec3(scale, scale, scale));

  instnode["mat"] = (unsigned long long)m;
  *minv = glm::inverse(*m);
  instnode["matInv"] = (unsigned long long)minv;
  *normi = glm::transpose(glm::inverse(glm::mat3(*m)));
  instnode["normi"] = (unsigned long long)normi;

  // transform mesh bounding box
  auto il = glm::vec3((*m) * glm::vec4(mbox->bounds_min, 1.f));
  auto ih = glm::vec3((*m) * glm::vec4(mbox->bounds_max, 1.f));
  Box3D * ibox = new gvt::render::data::primitives::Box3D(il, ih);
  instnode["bbox"] = (unsigned long long)ibox;
  instnode["centroid"] = ibox->centroid();
  cntxt->addToSync(instnode);

  cntxt->syncContext();
}
