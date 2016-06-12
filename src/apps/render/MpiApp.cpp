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

#include <sys/stat.h>
#include <tbb/task_scheduler_init.h>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <ply.h>

// timer
#include "gvt/core/utils/timer.h"

// database
#include "gvt/render/Types.h"
#include "gvt/render/RenderContext.h"
#include "gvt/render/data/primitives/Material.h"
#include "gvt/render/data/primitives/Mesh.h"
#include "gvt/render/data/scene/gvtCamera.h"

// async schedule
#include "gvt/render/unit/CommonWorks.h"
#include "gvt/render/unit/DomainTracer.h"
#include "gvt/render/unit/DomainWorks.h"
#include "gvt/render/unit/TestTracer.h"
#include "gvt/render/unit/Worker.h"

#ifndef MAX
#define MAX(a, b) ((a > b) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a, b) ((a < b) ? (a) : (b))
#endif

namespace apps {
namespace render {
namespace mpi {

using namespace gvt::render::data::primitives;

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

struct Options {
  enum TracerType {
    PING_TEST = 0,
    ASYNC_IMAGE,
    ASYNC_DOMAIN,
    SYNC_IMAGE,
    SYNC_DOMAIN,
    NUM_TRACERS
  };

  enum AdapterType { EMBREE, MANTA, OPTIX };

  int tracer = ASYNC_DOMAIN;
  int adapter = EMBREE;
  int width = 1920;
  int height = 1080;
  bool obj = false;
  int instanceCountX = 1;
  int instanceCountY = 1;
  int instanceCountZ = 1;
  int numFrames = 1;
  int numTbbThreads;
  std::string infile;
  glm::vec3 eye;
  glm::vec3 look;
};

void PrintUsage(const char *argv) {
  printf(
      "Usage : %s [-h] [-i <infile>] [-a <adapter>] [-n <x y z>] [-p] [-s "
      "<scheduler>] [-W <image_width>] [-H "
      "<image_height>] [-N <num_frames>] [-t <num_tbb_threads>]\n",
      argv);
  printf("  -h, --help\n");
  printf(
      "  -i, --infile <infile> (default: ../data/geom/bunny.obj for obj and "
      "./EnzoPlyData/Enzo8 for ply)\n");
  printf("  -a, --adapter <embree | manta | optix> (default: embree)\n");
  printf("  -t, --tracer <0-3> (default: 2)\n");
  printf(
      "      0: PING_TEST, 0: ASYNC_IMAGE, 1: ASYNC_DOMAIN, 2: SYNC_IMAGE, 3: "
      "SYNC_DOMAIN\n");
  printf(
      "  -n, --num-instances <x, y, z> specify the number of instances in each "
      "direction (default: 1 1 1). "
      "effective only with obj.\n");
  printf("  --obj use obj models (not ply models)\n");
  printf("  -W, --width <image_width> (default: 1920)\n");
  printf("  -H, --height <image_height> (default: 1080)\n");
  printf("  -N, --num-frames <num_frames> (default: 1)\n");
  printf("  --tbb <num_tbb_threads>\n");
  printf(
      "      (default: # cores for sync. schedulers or # cores - 2 for async. "
      "schedulers)\n");
}

void ParseCommandLine(int argc, char **argv, Options *options) {
  options->numTbbThreads = -1;
  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
      PrintUsage(argv[0]);
      exit(1);
    } else if (strcmp(argv[i], "-i") == 0 || strcmp(argv[i], "--infile") == 0) {
      options->infile = argv[++i];
      struct stat buf;
      if (stat(options->infile.c_str(), &buf) != 0) {
        printf("error: file not found. %s\n", options->infile.c_str());
        exit(1);
      }
    } else if (strcmp(argv[i], "-a") == 0 ||
               strcmp(argv[i], "--adapter") == 0) {
      ++i;
      if (strcmp(argv[i], "embree") == 0) {
        options->adapter = Options::EMBREE;
      } else if (strcmp(argv[i], "manta") == 0) {
        options->adapter = Options::MANTA;
      } else if (strcmp(argv[i], "optix") == 0) {
        options->adapter = Options::OPTIX;
      } else {
        printf("error: %s not defined\n", argv[i]);
        exit(1);
      }
    } else if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--tracer") == 0) {
      options->tracer = atoi(argv[++i]);
      if (options->tracer < 0 || options->tracer >= Options::NUM_TRACERS) {
        printf("error: %s not defined\n", argv[i]);
        exit(1);
      }
    } else if (strcmp(argv[i], "-n") == 0 ||
               strcmp(argv[i], "--num-instances") == 0) {
      options->instanceCountX = atoi(argv[++i]);
      options->instanceCountY = atoi(argv[++i]);
      options->instanceCountZ = atoi(argv[++i]);
      // } else if (strcmp(argv[i], "-p") == 0 || strcmp(argv[i], "--ply") == 0)
      // {
      //   options->ply = true;
    } else if (strcmp(argv[i], "--obj") == 0) {
      options->obj = true;
    } else if (strcmp(argv[i], "-W") == 0 || strcmp(argv[i], "--width") == 0) {
      options->width = atoi(argv[++i]);
    } else if (strcmp(argv[i], "-H") == 0 || strcmp(argv[i], "--height") == 0) {
      options->height = atoi(argv[++i]);
    } else if (strcmp(argv[i], "-N") == 0 ||
               strcmp(argv[i], "--num-frames") == 0) {
      options->numFrames = atoi(argv[++i]);
    } else if (strcmp(argv[i], "--tbb") == 0) {
      options->numTbbThreads = atoi(argv[++i]);
    } else {
      printf("error: %s not defined\n", argv[i]);
      exit(1);
    }
  }
  if (options->numTbbThreads <= 0) {
    if (options->tracer == 2 || options->tracer == 3) {
      options->numTbbThreads = MAX(1, std::thread::hardware_concurrency());
    } else {
      options->numTbbThreads = MAX(1, std::thread::hardware_concurrency() - 2);
    }
  }
  if (options->infile.empty()) {
    if (options->obj) {
      options->infile = std::string("../data/geom/bunny.obj");
    } else {
      options->infile = std::string("./EnzoPlyData/Enzo8/");
    }
  }
}

void CreateDatabase(const Options &options) {
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
  // rootdir = cmd.get<std::string>("file");
  rootdir = options.infile;  
  // rootdir = "/work/01197/semeraro/maverick/DAVEDATA/EnzoPlyData/";
  // filename = "/work/01197/semeraro/maverick/DAVEDATA/EnzoPlyData/block0.ply";
  // myfile = fopen(filename.c_str(),"r");
  // MPI_Init(&argc, &argv);
  // MPI_Pcontrol(0);
  // int rank = -1;
  // MPI_Comm_rank(MPI_COMM_WORLD, &rank);

  gvt::render::RenderContext *cntxt = gvt::render::RenderContext::instance();
  if (cntxt == NULL) {
    std::cout << "Something went wrong initializing the context" << std::endl;
    exit(0);
  }
  gvt::core::DBNodeH root = cntxt->getRootNode();
  gvt::core::DBNodeH dataNodes =
      cntxt->createNodeFromType("Data", "Data", root.UUID());
  gvt::core::DBNodeH instNodes =
      cntxt->createNodeFromType("Instances", "Instances", root.UUID());

  // Enzo isosurface...
  for (k = 0; k < 8; k++) {
    sprintf(txt, "%d", k);
    filename = "block";
    filename += txt;
    gvt::core::DBNodeH EnzoMeshNode =
        cntxt->createNodeFromType("Mesh", filename.c_str(), dataNodes.UUID());
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
      Material *m = new Material();
      m->type = LAMBERT;
      // m->type = EMBREE_MATERIAL_MATTE;
      m->kd = glm::vec3(1.0, 1.0, 1.0);
      m->ks = glm::vec3(1.0, 1.0, 1.0);
      m->alpha = 0.5;

      // m->type = EMBREE_MATERIAL_METAL;
      // copper metal
      m->eta = glm::vec3(.19, 1.45, 1.50);
      m->k = glm::vec3(3.06, 2.40, 1.88);
      m->roughness = 0.05;

      Mesh *mesh = new Mesh(m);
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
        mesh->addVertex(glm::vec3(vert->x, vert->y, vert->z));
      }
      glm::vec3 lower(xmin, ymin, zmin);
      glm::vec3 upper(xmax, ymax, zmax);
      Box3D *meshbbox = new gvt::render::data::primitives::Box3D(lower, upper);
      // add faces to mesh
      for (i = 0; i < nfaces; i++) {
        face = flist[i];
        mesh->addFace(face->verts[0] + 1, face->verts[1] + 1,
                      face->verts[2] + 1);
      }
      mesh->generateNormals();
      // add Enzo mesh to the database
      // EnzoMeshNode["file"] =
      // string("/work/01197/semeraro/maverick/DAVEDATA/EnzoPlyDATA/Block0.ply");
      EnzoMeshNode["file"] = std::string(filepath);
      EnzoMeshNode["bbox"] = (unsigned long long)meshbbox;
      EnzoMeshNode["ptr"] = (unsigned long long)mesh;
    }
    // add instance
    gvt::core::DBNodeH instnode =
        cntxt->createNodeFromType("Instance", "inst", instNodes.UUID());
    gvt::core::DBNodeH meshNode = EnzoMeshNode;
    Box3D *mbox = (Box3D *)meshNode["bbox"].value().toULongLong();
    instnode["id"] = k;
    instnode["meshRef"] = meshNode.UUID();
    auto m = new glm::mat4(1.f);
    auto minv = new glm::mat4(1.f);
    auto normi = new glm::mat3(1.f);
    instnode["mat"] = (unsigned long long)m;
    *minv = glm::inverse(*m);
    instnode["matInv"] = (unsigned long long)minv;
    *normi = glm::transpose(glm::inverse(glm::mat3(*m)));
    instnode["normi"] = (unsigned long long)normi;
    auto il = glm::vec3((*m) * glm::vec4(mbox->bounds_min, 1.f));
    auto ih = glm::vec3((*m) * glm::vec4(mbox->bounds_max, 1.f));
    Box3D *ibox = new gvt::render::data::primitives::Box3D(il, ih);
    instnode["bbox"] = (unsigned long long)ibox;
    instnode["centroid"] = ibox->centroid();
  }

  // add lights, camera, and film to the database
  gvt::core::DBNodeH lightNodes =
      cntxt->createNodeFromType("Lights", "Lights", root.UUID());
  gvt::core::DBNodeH lightNode =
      cntxt->createNodeFromType("PointLight", "conelight", lightNodes.UUID());
  lightNode["position"] = glm::vec3(512.0, 512.0, 2048.0);
  lightNode["color"] = glm::vec3(100.0, 100.0, 500.0);
  // camera
  gvt::core::DBNodeH camNode =
      cntxt->createNodeFromType("Camera", "conecam", root.UUID());
  camNode["eyePoint"] = glm::vec3(512.0, 512.0, 4096.0);
  camNode["focus"] = glm::vec3(512.0, 512.0, 0.0);
  camNode["upVector"] = glm::vec3(0.0, 1.0, 0.0);
  camNode["fov"] = (float)(25.0 * M_PI / 180.0);
  camNode["rayMaxDepth"] = (int)1;
  camNode["raySamples"] = (int)1;
  // film
  gvt::core::DBNodeH filmNode =
      cntxt->createNodeFromType("Film", "conefilm", root.UUID());
  // filmNode["width"] = 1900;
  // filmNode["height"] = 1080;
  filmNode["width"] = options.width;
  filmNode["height"] = options.height;

  //if (cmd.isSet("eye")) {
  //  std::vector<float> eye = cmd.getValue<float>("eye");
  //  camNode["eyePoint"] = glm::vec3(eye[0], eye[1], eye[2]);
  //}

  //if (cmd.isSet("look")) {
  //  std::vector<float> eye = cmd.getValue<float>("look");
  //  camNode["focus"] = glm::vec3(eye[0], eye[1], eye[2]);
  //}
  //if (cmd.isSet("wsize")) {
  //  std::vector<int> wsize = cmd.getValue<int>("wsize");
  //  filmNode["width"] = wsize[0];
  //  filmNode["height"] = wsize[1];
  //}

  gvt::core::DBNodeH schedNode =
      cntxt->createNodeFromType("Schedule", "Enzosched", root.UUID());
  // if (cmd.isSet("domain"))
  //   schedNode["type"] = gvt::render::scheduler::Domain;
  // else
  //   schedNode["type"] = gvt::render::scheduler::Image;
  if (options.tracer == Options::ASYNC_DOMAIN || options.tracer == Options::SYNC_DOMAIN) {
    schedNode["type"] = gvt::render::scheduler::Domain;
  } else {
    schedNode["type"] = gvt::render::scheduler::Image;
  }

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
  if (options.adapter == Options::EMBREE) {
    schedNode["adapter"] = gvt::render::adapter::Embree;
  } else if (options.adapter == Options::MANTA) {
    schedNode["adapter"] = gvt::render::adapter::Manta;
  } else if (options.adapter == Options::OPTIX) {
    schedNode["adapter"] = gvt::render::adapter::Optix;
  } else {
    schedNode["adapter"] = gvt::render::adapter::Embree;
  }

  // end db setup

  // use db to create structs needed by system

  // setup gvtCamera from database entries
  gvt::render::data::scene::gvtPerspectiveCamera mycamera;
  glm::vec3 cameraposition = camNode["eyePoint"].value().tovec3();
  glm::vec3 focus = camNode["focus"].value().tovec3();
  float fov = camNode["fov"].value().toFloat();
  glm::vec3 up = camNode["upVector"].value().tovec3();
  int rayMaxDepth = camNode["rayMaxDepth"].value().toInteger();
  int raySamples = camNode["raySamples"].value().toInteger();
  mycamera.lookAt(cameraposition, focus, up);
  mycamera.setMaxDepth(rayMaxDepth);
  mycamera.setSamples(raySamples);
  mycamera.setFOV(fov);
  mycamera.setFilmsize(filmNode["width"].value().toInteger(),
                       filmNode["height"].value().toInteger());

}  // void CreateDatabase(const Options& options) {

}  // namespace mpi
}  // namespace render
}  // namespace apps

using namespace apps::render::mpi;
using namespace gvt::render::unit;
using namespace gvt::core::time;

int main(int argc, char **argv) {
  int provided;
  MPI_Init_thread(&argc, &argv, MPI_THREAD_SINGLE, &provided);
  if (provided != MPI_THREAD_SINGLE) {
    std::cout << "error mpi_thread_single not available" << std::endl;
    exit(1);
  }
  int rank, size;
  MPI_Comm_rank(MPI_COMM_WORLD, &rank);
  MPI_Comm_size(MPI_COMM_WORLD, &size);
  { // warning: do not remove this curly brace
    timer t_database(false, "database timer:");

    Options options;
    ParseCommandLine(argc, argv, &options);

    // create a ray tracer
    RayTracer *tracer = NULL;
    if (options.tracer == Options::PING_TEST) {
      tracer = new PingTracer;
    } else if (options.tracer == Options::ASYNC_DOMAIN) {
      tracer = new DomainTracer;
    } else {
      std::cout << "rank " << rank << " error found unsupported tracer type "
                << options.tracer << std::endl;
      exit(1);
    }

    // create database
    if (options.tracer != Options::PING_TEST) {
      t_database.start();
      CreateDatabase(options);
      t_database.stop();
      std::cout << "rank " << rank << " created database. " << t_database << std::endl;
    }

    bool async = (options.tracer == Options::PING_TEST) ||
                 (options.tracer == Options::ASYNC_DOMAIN) ||
                 (options.tracer == Options::ASYNC_IMAGE);
    if (async) {
      // create a worker
      Worker worker(tracer);

      // register works
      Command::Register(&worker);
      PingTest::Register(&worker);
      RemoteRays::Register(&worker);

      // start the worker
      worker.Start(argc, argv);

      delete tracer;
    }
  } // warning: do not remove this curly brace
  MPI_Finalize();
}

