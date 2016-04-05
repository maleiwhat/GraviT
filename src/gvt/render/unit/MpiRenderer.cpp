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
#include "gvt/render/unit/Works.h"
#include "gvt/render/unit/PixelGatherWork.h"
#include "gvt/render/unit/TileLoadBalancer.h"

#include "gvt/render/algorithm/ImageTracer.h"
#include "gvt/render/algorithm/DomainTracer.h"

#include <iostream>
#include <mpi.h>
#include <boost/timer/timer.hpp>

#include <tbb/task_scheduler_init.h>
#include <thread>
 
#include <math.h>
#include <stdio.h>
#include <ply.h>

#include <queue>

// #define DEBUG_MPI_RENDERER
// #define DEBUG_RAYTX

using namespace std;
using namespace gvt::render;
using namespace gvt::core;
using namespace gvt::core::math;
using namespace gvt::core::mpi;
using namespace gvt::render::data::scene;
using namespace gvt::render::schedule;
using namespace gvt::render::data::primitives;
using namespace gvt::render::unit;

static boost::timer::cpu_timer t_run;

typedef struct Vertex {
  float x, y, z;
  float nx, ny, nz;
  void *other_props; /* other properties */
} Vertex;

typedef struct Face {
  unsigned char nverts; /* number of vertex indices in list */
  int *verts;           /* vertex index list */
  void *other_props;    /* other properties */
} Face;

PlyProperty vert_props[] = {
  /* list of property information for a vertex */
  { "x", Float32, Float32, offsetof(Vertex, x), 0, 0, 0, 0 },
  { "y", Float32, Float32, offsetof(Vertex, y), 0, 0, 0, 0 },
  { "z", Float32, Float32, offsetof(Vertex, z), 0, 0, 0, 0 },
  { "nx", Float32, Float32, offsetof(Vertex, nx), 0, 0, 0, 0 },
  { "ny", Float32, Float32, offsetof(Vertex, ny), 0, 0, 0, 0 },
  { "nz", Float32, Float32, offsetof(Vertex, nz), 0, 0, 0, 0 },
};

PlyProperty face_props[] = {
  /* list of property information for a face */
  { "vertex_indices", Int32, Int32, offsetof(Face, verts), 1, Uint8, Uint8, offsetof(Face, nverts) },
};
static Vertex **vlist;
static Face **flist;

#define MIN(a, b) ((a < b) ? (a) : (b))
#define MAX(a, b) ((a > b) ? (a) : (b))

MpiRenderer::MpiRenderer(int *argc, char ***argv)
    : Application(argc, argv), camera(NULL), image(NULL), dbOption(NULL), tileLoadBalancer(NULL), voter(NULL) {

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

void MpiRenderer::parseCommandLine(int argc, char **argv) {

  // TODO (hpark): generalize this
  this->dbOption = new TestDatabaseOption;
  TestDatabaseOption *option = static_cast<TestDatabaseOption *>(dbOption);

  if (argc > 1) {
    if (*argv[1] == 'i' || *argv[1] == 'I') {
      option->schedulerType = gvt::render::scheduler::Image;
    } else if (*argv[1] == 'd' || *argv[1] == 'D') {
      option->schedulerType = gvt::render::scheduler::Domain;
    }
    if (*argv[1] == 'I' || *argv[1] == 'D') {
      option->asyncMpi = false;
    }
  }
  if (argc > 4) {
    option->instanceCountX = atoi(argv[2]);
    option->instanceCountY = atoi(argv[3]);
    option->instanceCountZ = atoi(argv[4]);
  }
  if (argc > 6) {
    option->filmWidth = atoi(argv[5]);
    option->filmHeight = atoi(argv[6]);
  }
  if (argc > 7) {
    if (*argv[7] == 'o') { // obj
      option->ply = false;
    }
    // option->numFrames = atoi(argv[7]);
  }
}

void MpiRenderer::createDatabase() {
  TestDatabaseOption *option = static_cast<TestDatabaseOption *>(dbOption);
  if (option->ply) {
    makePlyDatabase();
  } else {
    makeObjDatabase();
  }
}

void MpiRenderer::makePlyDatabase() {
  TestDatabaseOption *option = static_cast<TestDatabaseOption *>(dbOption);
  // mess I use to open and read the ply file with the c utils I found.
  PlyFile *in_ply;
  Vertex *vert;
  Face *face;
  int elem_count, nfaces, nverts;
  int i, j, k;
  float xmin, ymin, zmin, xmax, ymax, zmax;
  char *elem_name;
  ;
  FILE *myfile;
  char txt[16];
  std::string temp;
  std::string filename, filepath, rootdir;
  // rootdir = "/Users/jbarbosa/r/EnzoPlyData/";
  rootdir = "/work/01197/semeraro/maverick/DAVEDATA/EnzoPlyData/";
  // filename = "/work/01197/semeraro/maverick/DAVEDATA/EnzoPlyData/block0.ply";
  // myfile = fopen(filename.c_str(),"r");
  // MPI_Init(&argc, &argv);
  // MPI_Pcontrol(0);
  // int rank = -1;
  // MPI_Comm_rank(MPI_COMM_WORLD, &rank);

  // gvt::render::RenderContext *renderContext = gvt::render::RenderContext::instance();
  // if (renderContext == NULL) {
  //   std::cout << "Something went wrong initializing the context" << std::endl;
  //   exit(0);
  // }
  gvt::core::DBNodeH root = renderContext->getRootNode();
  gvt::core::DBNodeH dataNodes = renderContext->createNodeFromType("Data", "Data", root.UUID());
  gvt::core::DBNodeH instNodes = renderContext->createNodeFromType("Instances", "Instances", root.UUID());

  // // create data node
  // Uuid dataNodeId = createNode("Data", "Data");
  // Uuid instancesNodeId = createNode("Instances", "Instances");

  // Enzo isosurface...
  const int numPlyFiles = 8;
  for (k = 0; k < numPlyFiles; k++) {
    sprintf(txt, "%d", k);
    filename = "block";
    filename += txt;
    gvt::core::DBNodeH plyMeshNode = renderContext->createNodeFromType("Mesh", filename.c_str(), dataNodes.UUID());
    // read in some ply data and get ready to load it into the mesh
    // filepath = rootdir + "block" + std::string(txt) + ".ply";
    filepath = rootdir + filename + ".ply";
    myfile = fopen(filepath.c_str(), "r");
    in_ply = read_ply(myfile);
    for (i = 0; i < in_ply->num_elem_types; i++) {
      elem_name = setup_element_read_ply(in_ply, i, &elem_count);
      temp = elem_name;
      if (temp == "vertex") {
        vlist = (Vertex **)malloc(sizeof(Vertex *) * elem_count);
        nverts = elem_count;
        setup_property_ply(in_ply, &vert_props[0]);
        setup_property_ply(in_ply, &vert_props[1]);
        setup_property_ply(in_ply, &vert_props[2]);
        for (j = 0; j < elem_count; j++) {
          vlist[j] = (Vertex *)malloc(sizeof(Vertex));
          get_element_ply(in_ply, (void *)vlist[j]);
        }
      } else if (temp == "face") {
        flist = (Face **)malloc(sizeof(Face *) * elem_count);
        nfaces = elem_count;
        setup_property_ply(in_ply, &face_props[0]);
        for (j = 0; j < elem_count; j++) {
          flist[j] = (Face *)malloc(sizeof(Face));
          get_element_ply(in_ply, (void *)flist[j]);
        }
      }
    }
    close_ply(in_ply);
    // smoosh data into the mesh object
    {
      Mesh *mesh = new Mesh(new Lambert(Vector4f(1.0, 1.0, 1.0, 1.0)));
      vert = vlist[0];
      xmin = vert->x;
      ymin = vert->y;
      zmin = vert->z;
      xmax = vert->x;
      ymax = vert->y;
      zmax = vert->z;

      for (i = 0; i < nverts; i++) {
        vert = vlist[i];
        xmin = MIN(vert->x, xmin);
        ymin = MIN(vert->y, ymin);
        zmin = MIN(vert->z, zmin);
        xmax = MAX(vert->x, xmax);
        ymax = MAX(vert->y, ymax);
        zmax = MAX(vert->z, zmax);
        mesh->addVertex(Point4f(vert->x, vert->y, vert->z, 1.0));
      }
      Point4f lower(xmin, ymin, zmin);
      Point4f upper(xmax, ymax, zmax);
      Box3D *meshbbox = new gvt::render::data::primitives::Box3D(lower, upper);
      // add faces to mesh
      for (i = 0; i < nfaces; i++) {
        face = flist[i];
        mesh->addFace(face->verts[0] + 1, face->verts[1] + 1, face->verts[2] + 1);
      }
      mesh->generateNormals();
      // add Enzo mesh to the database
      // plyMeshNode["file"] = string("/work/01197/semeraro/maverick/DAVEDATA/EnzoPlyDATA/Block0.ply");
      plyMeshNode["file"] = string(filepath);
      plyMeshNode["bbox"] = (unsigned long long)meshbbox;
      plyMeshNode["ptr"] = (unsigned long long)mesh;
    }
    // add instance
    gvt::core::DBNodeH instnode = renderContext->createNodeFromType("Instance", "inst", instNodes.UUID());
    gvt::core::DBNodeH meshNode = plyMeshNode;
    Box3D *mbox = (Box3D *)meshNode["bbox"].value().toULongLong();
    instnode["id"] = k;
    instnode["meshRef"] = meshNode.UUID();
    auto m = new gvt::core::math::AffineTransformMatrix<float>(true);
    auto minv = new gvt::core::math::AffineTransformMatrix<float>(true);
    auto normi = new gvt::core::math::Matrix3f();
    instnode["mat"] = (unsigned long long)m;
    *minv = m->inverse();
    instnode["matInv"] = (unsigned long long)minv;
    *normi = m->upper33().inverse().transpose();
    instnode["normi"] = (unsigned long long)normi;
    auto il = (*m) * mbox->bounds[0];
    auto ih = (*m) * mbox->bounds[1];
    Box3D *ibox = new gvt::render::data::primitives::Box3D(il, ih);
    instnode["bbox"] = (unsigned long long)ibox;
    instnode["centroid"] = ibox->centroid();
  }

  // add lights, camera, and film to the database
  gvt::core::DBNodeH lightNodes = renderContext->createNodeFromType("Lights", "Lights", root.UUID());
  gvt::core::DBNodeH lightNode = renderContext->createNodeFromType("PointLight", "conelight", lightNodes.UUID());
  lightNode["position"] = Vector4f(512.0, 512.0, 2048.0, 0.0);
  lightNode["color"] = Vector4f(1.0, 1.0, 1.0, 0.0);

  // camera
  Point4f eye(512.0, 512.0, 4096.0, 1.0);
  Point4f focus(512.0, 512.0, 0.0, 1.0);
  Vector4f upVector(0.0, 1.0, 0.0, 0.0);
  float fov = (float)(25.0 * M_PI / 180.0);
  Uuid cameraNodeId = createCameraNode(eye, focus, upVector, fov, option->filmWidth, option->filmHeight);

  // gvt::core::DBNodeH camNode = renderContext->createNodeFromType("Camera", "conecam", root.UUID());
  // camNode["eyePoint"] = Point4f(512.0, 512.0, 4096.0, 1.0);
  // camNode["focus"] = Point4f(512.0, 512.0, 0.0, 1.0);
  // camNode["upVector"] = Vector4f(0.0, 1.0, 0.0, 0.0);
  // camNode["fov"] = (float)(25.0 * M_PI / 180.0);
  // film
  Uuid filmNodeId = createFilmNode(option->filmWidth, option->filmHeight, "");

  // gvt::core::DBNodeH filmNode = renderContext->createNodeFromType("Film", "conefilm", root.UUID());
  // filmNode["width"] = option->filmWidth;
  // filmNode["height"] = option->filmHeight;
  // filmNode["width"] = 2000;
  // filmNode["height"] = 2000;

//   gvt::core::DBNodeH schedNode = renderContext->createNodeFromType("Schedule", "Enzosched", root.UUID());
//   schedNode["type"] = gvt::render::scheduler::Image;
// // schedNode["type"] = gvt::render::scheduler::Domain;

// #ifdef GVT_RENDER_ADAPTER_EMBREE
//   int adapterType = gvt::render::adapter::Embree;
// #elif GVT_RENDER_ADAPTER_MANTA
//   int adapterType = gvt::render::adapter::Manta;
// #elif GVT_RENDER_ADAPTER_OPTIX
//   int adapterType = gvt::render::adapter::Optix;
// #elif
//   GVT_DEBUG(DBG_ALWAYS, "ERROR: missing valid adapter");
// #endif

  // schedNode["adapter"] = gvt::render::adapter::Embree;

  Uuid scheduleNodeId = createScheduleNode(option->schedulerType, option->adapterType);

  // end db setup

  // use db to create structs needed by system

  // setup gvtCamera from database entries
  // gvtPerspectiveCamera mycamera;
  // Point4f cameraposition = camNode["eyePoint"].value().toPoint4f();
  // Point4f focus = camNode["focus"].value().toPoint4f();
  // float fov = camNode["fov"].value().toFloat();
  // Vector4f up = camNode["upVector"].value().toVector4f();
  // mycamera.lookAt(cameraposition, focus, up);
  // mycamera.setFOV(fov);
  // mycamera.setFilmsize(filmNode["width"].value().toInteger(), filmNode["height"].value().toInteger());

// #ifdef GVT_USE_MPE
//   MPE_Log_event(readend, 0, NULL);
// #endif
  // setup image from database sizes
}

void MpiRenderer::makeObjDatabase() {
  // create data node
  Uuid dataNodeId = createNode("Data", "Data");

  std::string objName("bunny"); // TODO: fixed for now

  // // add a mesh
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

  TestDatabaseOption *option = static_cast<TestDatabaseOption *>(dbOption);
  int instanceCountX = option->instanceCountX;
  int instanceCountY = option->instanceCountY;
  int instanceCountZ = option->instanceCountZ;

  Vector3f minPos((extent[0] + gapX) * instanceCountX * -0.5f, (extent[1] + gapY) * instanceCountY * -0.5f,
                  (extent[2] + gapZ) * instanceCountZ * -0.5f);

  int instanceId = 0;
  for (int z = 0; z < instanceCountZ; ++z) {
    for (int y = 0; y < instanceCountY; ++y) {
      for (int x = 0; x < instanceCountX; ++x) {
        // add a mesh
        Uuid meshNodeId = addMesh(dataNodeId, objName + "_mesh", "../data/geom/" + objName + ".obj");

        // int instanceId = y * 2 + x;
        // m: transform matrix
        auto m = new gvt::core::math::AffineTransformMatrix<float>(true);
        *m = *m * gvt::core::math::AffineTransformMatrix<float>::createTranslation(minPos[0] + x * (extent[0] + gapX),
                                                                                   minPos[1] + y * (extent[1] + gapY),
                                                                                   minPos[2] + z * (extent[2] + gapZ));
        *m = *m * gvt::core::math::AffineTransformMatrix<float>::createScale(1.0f, 1.0f, 1.0f);
        Uuid instanceUuid = addInstance(instancesNodeId, meshNodeId, instanceId++, objName, m);
      }
    }
  }

  // create lights node
  Uuid lightsNodeId = createNode("Lights", "Lights");

  // add lights
  Vector4f lightPosition(0.0, 0.1, 0.5, 0.0);
  Vector4f lightColor(1.0, 1.0, 1.0, 0.0);
  Uuid lightNodeId = addPointLight(lightsNodeId, "point_light", lightPosition, lightColor);

  // create camera node
  Point4f eye(0.0, 0.5, 1.2, 1.0);
  Point4f focus(0.0, 0.0, 0.0, 1.0);
  Vector4f upVector(0.0, 1.0, 0.0, 0.0);
  float fov = (45.0 * M_PI / 180.0);
  // const unsigned int width = 1920;
  // const unsigned int height = 1080;
  const unsigned int width = option->filmWidth;
  const unsigned int height = option->filmHeight;
  Uuid cameraNodeId = createCameraNode(eye, focus, upVector, fov, width, height);

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
  // adapterType = gvt::render::adapter::Manta;
  Uuid scheduleNodeId = createScheduleNode(option->schedulerType, option->adapterType);
}

bool MpiRenderer::isNodeTypeReserved(const std::string &type) {
  return ((type == std::string("Camera")) || (type == std::string("Film")) || (type == std::string("View")) ||
          (type == std::string("Dataset")) || (type == std::string("Attribute")) || (type == std::string("Mesh")) ||
          (type == std::string("Instance")) || (type == std::string("PointLight")) ||
          (type == std::string("Schedule")));
}

DBNodeH MpiRenderer::getNode(const Uuid &id) { return renderContext->getNode(id); }

Uuid MpiRenderer::createNode(const std::string &type, const std::string &name) {
  if (isNodeTypeReserved(type)) {
    std::cout << "error: reserved node type " << type << std::endl;
    exit(1);
  }
  gvt::core::DBNodeH root = renderContext->getRootNode();
  gvt::core::DBNodeH node = renderContext->createNodeFromType(type, name, root.UUID());
  return node.UUID();
}

Uuid MpiRenderer::addMesh(const Uuid &parentNodeId, const std::string &meshName, const std::string &objFilename) {
  gvt::core::DBNodeH node = renderContext->createNodeFromType("Mesh", meshName, parentNodeId);
  gvt::render::data::domain::reader::ObjReader objReader(objFilename);
  Mesh *mesh = objReader.getMesh();
  mesh->generateNormals();
  mesh->computeBoundingBox();
  Box3D *meshbbox = mesh->getBoundingBox();
  // add mesh to the database
  node["file"] = objFilename;
  node["bbox"] = (unsigned long long)meshbbox;
  node["ptr"] = (unsigned long long)mesh;
  return node.UUID();
}

Box3D MpiRenderer::getMeshBounds(const std::string &objFilename) {
  gvt::render::data::domain::reader::ObjReader objReader(objFilename);
  Mesh *mesh = objReader.getMesh();
  // mesh->generateNormals();
  mesh->computeBoundingBox();
  Box3D *bounds = mesh->getBoundingBox();
  return *bounds;
}

Box3D MpiRenderer::getMeshBounds(const Uuid &id) {
  gvt::core::DBNodeH meshNode = renderContext->getNode(id);
  // Box3D* bounds = *gvt::core::variant_toBox3DPtr(meshNode["bbox"].value());
  Box3D *bounds = (Box3D *)meshNode["bbox"].value().toULongLong();
  return *bounds;
}

Uuid MpiRenderer::addInstance(const Uuid &parentNodeId, const Uuid &meshId, int instanceId,
                              const std::string &instanceName,
                              gvt::core::math::AffineTransformMatrix<float> *transform) {

  gvt::core::DBNodeH node = renderContext->createNodeFromType("Instance", instanceName, parentNodeId);

  gvt::core::DBNodeH meshNode = renderContext->getNode(meshId);
  // Box3D* mbox = *gvt::core::variant_toBox3DPtr(meshNode["bbox"].value());
  Box3D *mbox = (Box3D *)meshNode["bbox"].value().toULongLong();

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
  printf("[new instance %d] bounds: min(%.3f %.3f %.3f), max(%.3f %.3f %.3f)\n", instanceId, il[0], il[1], il[2], ih[0],
         ih[1], ih[2]);
#endif

  return node.UUID();
}

Uuid MpiRenderer::addPointLight(const Uuid &parentNodeId, const std::string &lightName, const Vector4f &position,
                                const Vector4f &color) {
  gvt::core::DBNodeH node = renderContext->createNodeFromType("PointLight", lightName, parentNodeId);
  node["position"] = Vector4f(0.0, 0.1, 0.5, 0.0);
  node["color"] = Vector4f(1.0, 1.0, 1.0, 0.0);
  return node.UUID();
}

Uuid MpiRenderer::createCameraNode(const Point4f &eye, const Point4f &focus, const Vector4f &upVector, float fov,
                                   unsigned int width, unsigned int height) {

  gvt::core::DBNodeH root = renderContext->getRootNode();

  gvt::core::DBNodeH node = renderContext->createNodeFromType("Camera", "cam", root.UUID());

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

Uuid MpiRenderer::createFilmNode(int width, int height, const std::string &sceneName) {

  gvt::core::DBNodeH root = renderContext->getRootNode();

  gvt::core::DBNodeH node = renderContext->createNodeFromType("Film", "film", root.UUID());

  node["width"] = width;
  node["height"] = height;

  // image = new Image(width, height, sceneName);

  return node.UUID();
}

Uuid MpiRenderer::createScheduleNode(int schedulerType, int adapterType) {

  gvt::core::DBNodeH root = renderContext->getRootNode();

  gvt::core::DBNodeH node = renderContext->createNodeFromType("Schedule", "sched", root.UUID());

  node["type"] = schedulerType;
  node["adapter"] = adapterType;

  return node.UUID();
}

void MpiRenderer::initInstanceRankMap() {

  gvt::core::Vector<gvt::core::DBNodeH> dataNodes = root["Data"].getChildren();

#ifdef DEBUG_MPI_RENDERER
  std::cout << "instance node size: " << instanceNodes.size() << "\ndata node size: " << dataNodes.size() << "\n";
#endif

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
    GVT_DEBUG(DBG_ALWAYS, "[" << GetRank() << "] domain scheduler: instId: " << i << ", dataIdx: " << dataIdx
                           << ", target mpi node: " << ownerRank << ", world size: " << GetSize());
#ifdef DEBUG_MPI_RENDERER
    std::cout << "[" << GetRank() << "] domain scheduler: instId: " << i << ", dataIdx: " << dataIdx
              << ", target mpi node: " << ownerRank << ", world size: " << GetSize() << "\n";
#endif
    GVT_ASSERT(dataIdx != (size_t)-1, "domain scheduler: could not find data node");
    instanceRankMap[instanceNodes[i].UUID()] = ownerRank;
  }
}

void MpiRenderer::setupRender() {

  tbb::task_scheduler_init init(std::thread::hardware_concurrency());

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

  numRanks = GetSize();
  myRank = GetRank();
   
  if (schedType == scheduler::Image)  {
    if (GetRank() == rank::Server) {
      serverReady = false; 
      pthread_mutex_init(&serverReadyLock, NULL);
      pthread_cond_init(&serverReadyCond, NULL);
    }
  } else if (schedType == scheduler::Domain) {
    initInstanceRankMap();
    pthread_mutex_init(&rayTransferBufferLock, NULL);
    pthread_mutex_init(&rayTransferMutex, NULL);
    if (numRanks > 1) {
      voter = new Voter(numRanks, myRank, &rayQueue);
    }
  }
  // image write
  imageReady = false; 
  pthread_mutex_init(&imageReadyLock, NULL);
  pthread_cond_init(&imageReadyCond, NULL);
}

void MpiRenderer::freeRender() {
#ifdef DEBUG_MPI_RENDERER
  printf("Rank %d: MpiRenderer::freeRender. cleaning up.\n", GetRank());
#endif
  delete acceleration;
  delete[] rayQueueMutex;
  delete[] colorBufMutex;
  if (voter) delete voter;
}

void MpiRenderer::render() {

  TestDatabaseOption *option = static_cast<TestDatabaseOption *>(dbOption);

  setupRender();
  int schedType = root["Schedule"]["type"].value().toInteger();
  int rank = GetRank();

  if (option->asyncMpi) {
    // TODO (hpark):
    // collapse the following two if-else blocks into a single block
    if (schedType == scheduler::Image) { // image scheduler with mpi layer

      if (rank == 0)
        printf("[async mpi] starting image scheduler using %d processes\n", GetSize());

      RequestWork::Register();
      TileWork::Register();
      ImageTileWork::Register();
      PixelWork::Register();

      Start();

      if (rank == rank::Server) {
        initServer();
      }

      RequestWork request;
      request.setSourceRank(GetRank());
      request.Send(rank::Server);

      Wait();

    } else { // domain scheduler with mpi layer
      if (rank == 0)
        printf("[async mpi] starting domain scheduler using %d processes\n", GetSize());

      DomainTileWork::Register();
      RayTransferWork::Register();
      VoteWork::Register();
      PixelGatherWork::Register();

      Start();

      image = new Image(imageWidth, imageHeight, "image");
      DomainTileWork work;
      work.setTileSize(0, 0, imageWidth, imageHeight);

      const int numFrames = 30;

      for (int i = 0; i < numFrames; ++i) {
        work.Action();
        pthread_mutex_lock(&imageReadyLock);
        while(!this->imageReady) {
          pthread_cond_wait(&imageReadyCond, &imageReadyLock);
        }
        imageReady = false;
        pthread_mutex_unlock(&imageReadyLock);
        printf("[async mpi] domain scheduler frame %d done\n", i);
      }
      
      Kill();
    }

    freeRender();

  } else { // without mpi layer

    // int rank = GetRank();
    // setupRender();
    camera->AllocateCameraRays();
    camera->generateRays();
    image = new Image(imageWidth, imageHeight, "image");

    if (schedType == scheduler::Image) {
      if (rank == 0)
        printf("[sync mpi] starting image scheduler without the mpi layer using %d processes\n", GetSize());
      gvt::render::algorithm::Tracer<ImageScheduler>(camera->rays, *image)();
    } else {
      if (rank == 0)
        printf("[sync mpi] starting domain scheduler without the mpi layer using %d processes\n", GetSize());
      gvt::render::algorithm::Tracer<DomainScheduler>(camera->rays, *image)();
    }

    image->Write();
    freeRender();

    Quit::Register();
    Start();
    if (rank == 0) {
      Quit quit;
      quit.Broadcast(true, true);
    }
  }
}

void MpiRenderer::initServer() {
  // TODO (hpark): For now, equally divide the image
  // try coarser/finer granularity later
  int schedType = root["Schedule"]["type"].value().toInteger();
  // int numRanks = GetSize();
  int numWorkers = numRanks;
  int granularity = numRanks;

  tileLoadBalancer = new TileLoadBalancer(schedType, imageWidth, imageHeight, granularity, numWorkers);
  image = new Image(imageWidth, imageHeight, "image");
  pendingPixelCount = imageWidth * imageHeight;

  pthread_mutex_lock(&serverReadyLock);
  serverReady = true;
  pthread_cond_signal(&serverReadyCond);
  pthread_mutex_unlock(&serverReadyLock);
}

void MpiRenderer::aggregatePixel(int pixelId, const GVT_COLOR_ACCUM &addend) {
  GVT_COLOR_ACCUM &color = framebuffer[pixelId];
  color.add(addend);
  color.rgba[3] = 1.f;
  color.clamp();
}

bool MpiRenderer::transferRays() {
  if (numRanks > 1) {
    sendRays();
    receiveRays();
    return voter->updateState();
  } else {
    return rayQueue.empty(); 
  }
}

void MpiRenderer::sendRays() {
  for (auto &q : rayQueue) {
    int instance = q.first;
    RayVector& rays = q.second; 
    int ownerRank = getInstanceOwner(instance);
    size_t numRaysToSend = rays.size();
    if (ownerRank != myRank && numRaysToSend > 0) {
      voter->addNumPendingRays(numRaysToSend);
      RayTransferWork work;
      work.setup(RayTransferWork::Request, myRank, instance, &rays);
      work.Send(ownerRank);
#ifdef DEBUG_RAYTX
      printf("rank %d: sent %lu rays instance %d\n", myRank, numRaysToSend, instance);
#endif
    }
  }
}

void MpiRenderer::receiveRays() {
  pthread_mutex_lock(&rayTransferBufferLock);
  for (size_t i = 0; i < rayTransferBuffer.size(); ++i) {
    RayTransferWork* raytx = rayTransferBuffer[i];
    raytx->copyIncomingRays(&rayQueue);

    RayTransferWork grant;
    grant.setup(RayTransferWork::Grant, myRank, raytx->getNumRays());
    grant.Send(raytx->getSenderRank());
#ifdef DEBUG_RAYTX
    printf("rank %d: recved %d rays instance %d \n", myRank, raytx->getNumRays(), raytx->getInstanceId());
#endif
    delete raytx;
  }
  rayTransferBuffer.clear(); // TODO: avoid this

  pthread_mutex_unlock(&rayTransferBufferLock);
}

void MpiRenderer::bufferRayTransferWork(RayTransferWork* work) {
  pthread_mutex_lock(&rayTransferBufferLock);
  rayTransferBuffer.push_back(work); // TODO: avoid resizing
  pthread_mutex_unlock(&rayTransferBufferLock);
}

void MpiRenderer::bufferVoteWork(VoteWork* work) {
  voter->bufferVoteWork(work);
}

void MpiRenderer::voteForResign(int senderRank, unsigned int timeStamp) {
  voter->voteForResign(senderRank, timeStamp);
}

void MpiRenderer::voteForNoWork(int senderRank, unsigned int timeStamp) {
  voter->voteForNoWork(senderRank, timeStamp);
}

void MpiRenderer::applyRayTransferResult(int numRays) {
  voter->subtractNumPendingRays(numRays);
}

void MpiRenderer::applyVoteResult(int voteType, unsigned int timeStamp) {
  voter->applyVoteResult(voteType, timeStamp);
}

void MpiRenderer::copyIncomingRays(int instanceId, const gvt::render::actor::RayVector *incomingRays) {
  pthread_mutex_lock(&rayTransferMutex);
  if (rayQueue.find(instanceId) != rayQueue.end()) {
    rayQueue[instanceId].insert(rayQueue[instanceId].end(), incomingRays->begin(), incomingRays->end()); 
  } else {
    rayQueue[instanceId] = *incomingRays;
  }
  pthread_mutex_unlock(&rayTransferMutex);
}

Voter::Voter(int numRanks, int myRank, std::map<int, gvt::render::actor::RayVector> *rayQ)
  : numRanks(numRanks),
    myRank(myRank),
    rayQ(rayQ),
    state(WaitForNoWork),
    numPendingRays(0),
    validTimeStamp(0),
    votesAvailable(false),
    resignGrant(false),
    numVotesReceived(0),
    commitCount(0),
    numPendingVotes(0) {
    
  pthread_mutex_init(&votingLock, NULL);
  // pthread_mutex_init(&voteWorkBufferLock, NULL);
}

void Voter::addNumPendingRays(int n) {
  pthread_mutex_lock(&votingLock);
  numPendingRays += n;
  pthread_mutex_unlock(&votingLock);
}

void Voter::subtractNumPendingRays(int n) {
  pthread_mutex_lock(&votingLock);
#ifdef DEBUG_RAYTX
  printf("rank %d: Voter::subtractNumPendingRays: numPendingRays(before) %d numPendingRays(after) %d\n", myRank, numPendingRays, (numPendingRays-n));
#endif
  numPendingRays -= n;
  assert(numPendingRays >= 0);
  pthread_mutex_unlock(&votingLock);
}

void Voter::bufferVoteWork(VoteWork* work) {
  pthread_mutex_lock(&voteWorkBufferLock);
  voteWorkBuffer.push_back(work); // TODO: avoid resizing
#ifdef DEBUG_RAYTX
  printf("rank %d: Voter::bufferVoteWork received vote request from rank %d voteType %d timeStamp %d\n", myRank, work->getSenderRank(), work->getVoteType(), work->getTimeStamp());
#endif
  pthread_mutex_unlock(&voteWorkBufferLock);
}

void Voter::voteForResign(int senderRank, unsigned int timeStamp) {
  int vote = (state == WaitForResign || state == Resigned) ?  VoteWork::Commit : VoteWork::Abort;
  VoteWork grant;
  grant.setup(vote, myRank, timeStamp);
  grant.Send(senderRank);
}

void Voter::voteForNoWork(int senderRank, unsigned int timeStamp) {
  int vote = (state != WaitForNoWork) ?  VoteWork::Commit : VoteWork::Abort;
  VoteWork grant;
  grant.setup(vote, myRank, timeStamp);
  grant.Send(senderRank);
}
void Voter::applyVoteResult(int voteType, unsigned int timeStamp) {
  pthread_mutex_lock(&votingLock);
  --numPendingVotes;
  assert(numPendingVotes >= 0);
  if (timeStamp == validTimeStamp) {
    ++numVotesReceived;
    commitCount += (voteType == VoteWork::Commit);
    assert(numVotesReceived < numRanks);
    if (numVotesReceived == numRanks - 1) {
      votesAvailable = true;
#ifdef DEBUG_RAYTX
      printf("rank %d: Voter::applyVoteResult: votesAvailable %d, numPendingVotes %d, numVotesReceived %d, commitCount %d voteType %d\n", myRank, votesAvailable, numPendingVotes, numVotesReceived, commitCount, voteType);
#endif
    }
  }
  pthread_mutex_unlock(&votingLock);
}

bool Voter::updateState() {
  pthread_mutex_lock(&votingLock);

  bool hasWork = !(rayQ->empty() && numPendingRays == 0);
  bool allDone = false;

  switch(state) {
    case WaitForNoWork: { // has work
      if (!hasWork) {
        requestForVotes(VoteWork::NoWork, validTimeStamp);
        state = WaitForVotes;
#ifdef DEBUG_RAYTX
        printf("rank %d: WaitForNoWork -> WaitForVotes\n", myRank);
#endif
      }
    }
    break;
    case WaitForVotes: { // pending votes
      if (hasWork) {
        ++validTimeStamp;
        numVotesReceived = 0;
        commitCount = 0;
        votesAvailable = false;
        state = WaitForNoWork;
#ifdef DEBUG_RAYTX
        printf("rank %d: WaitForVotes -> WaitForNoWork\n", myRank);
#endif
      } else if (votesAvailable) {
        bool commit = checkVotes();

        numVotesReceived = 0;
        commitCount = 0;
        votesAvailable = false;

        if (commit) {
          requestForVotes(VoteWork::Resign, validTimeStamp);
          state = WaitForResign;
#ifdef DEBUG_RAYTX
        printf("rank %d: updateState(): WaitForVotes -> WaitForResign\n", myRank);
#endif
        } else { 
#ifdef DEBUG_RAYTX
        printf("rank %d: updateState(): abort message received. retrying requestForVotes.\n", myRank);
#endif
          // TODO: hpark adjust request interval based on workloads?
          requestForVotes(VoteWork::NoWork, validTimeStamp);
        }
      }
    }
    break;
    case WaitForResign: {
      if (hasWork) { // TODO: hpark possible? can't just think of this case
        assert(false); // TODO: hpark let's disable this for now
        ++validTimeStamp;
        numVotesReceived = 0;
        commitCount = 0;
        votesAvailable = false;
        state = WaitForNoWork;

      } else if (votesAvailable) {
        bool commit = checkVotes();

        numVotesReceived = 0;
        commitCount = 0;
        votesAvailable = false;

        if (commit) {
          allDone = true;
          state = Resigned;
#ifdef DEBUG_RAYTX
        printf("rank %d: WaitForResign allDone=true\n", myRank);
#endif
        } else {
          requestForVotes(VoteWork::Resign, validTimeStamp);
        }
      }
    }
    break;
    case Resigned: {
    }
    break;
    default: {
      state = WaitForNoWork;
    }
    break;
  }
  // vote();
  pthread_mutex_unlock(&votingLock);
  return allDone;
}

void Voter::vote() {
  pthread_mutex_lock(&voteWorkBufferLock);
  for (size_t i = 0; i < voteWorkBuffer.size(); ++i) {
    VoteWork* request = voteWorkBuffer[i];
#ifdef DEBUG_RAYTX
  printf("rank %d: processing vote request type=%d timeStamp=%d\n", myRank, request->getVoteType(), request->getTimeStamp());
#endif
    int vote;
    int type = request->getVoteType();
    if (type == VoteWork::NoWork) {
      vote = (state != WaitForNoWork) ?  VoteWork::Commit : VoteWork::Abort;
    // } else if (type == VoteWork::Resign) {
    //   vote = (state == WaitForResign) ?  VoteWork::Commit : VoteWork::Abort;
    } else {
      assert(false);
    }
    VoteWork grant;
    grant.setup(vote, myRank, request->getTimeStamp());
    grant.Send(request->getSenderRank());
#ifdef DEBUG_RAYTX
    printf("rank %d: sent vote voteType %d timeStamp %d to rank %d in response to vote Type %d (state %d numPendingVotes %d)\n", myRank, vote, request->getTimeStamp(), request->getSenderRank(), type, state, numPendingVotes);
#endif
    delete request;
  }
  voteWorkBuffer.clear(); // TODO: avoid this
  pthread_mutex_unlock(&voteWorkBufferLock);
}

bool Voter::checkVotes() {
  return (commitCount == numRanks - 1);
}

void Voter::requestForVotes(int voteType, unsigned int timeStamp) {
  numPendingVotes += (numRanks - 1);
  for (int i = 0; i < numRanks; ++i) {
    if (i != myRank) {
      VoteWork work;
      work.setup(voteType, myRank, timeStamp);
      work.Send(i);
#ifdef DEBUG_RAYTX
      printf("rank %d: sent vote request to rank %d voteType %d timeStamp %d\n", myRank, i, voteType, timeStamp);
#endif
    }
  }
}

