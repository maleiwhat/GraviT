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

#include <cstdlib>
#include <glob.h>
#include <iostream>
#include <map>
#include <sstream>
#include <sys/stat.h>
#include <tbb/task_scheduler_init.h>
#include <thread>
#include <glm/glm.hpp>

// timer
#include "gvt/core/utils/timer.h"

// database
#include "gvt/render/RenderContext.h"
#include "gvt/render/Types.h"
#include "gvt/render/data/primitives/Material.h"
#include "gvt/render/data/primitives/Mesh.h"
#include "gvt/render/data/scene/Image.h"
#include "gvt/render/data/scene/gvtCamera.h"

// common
#include "gvt/render/unit/Types.h"

// async schedule
#include "gvt/render/unit/CommonWorks.h"
#include "gvt/render/unit/DomainTracer.h"
#include "gvt/render/unit/DomainWorks.h"
#include "gvt/render/unit/Profiler.h"
#include "gvt/render/unit/TestTracer.h"
#include "gvt/render/unit/Worker.h"

// sync schedule
#include "gvt/render/algorithm/DomainTracer.h"

#include "apps/render/MpiApp.h"

// warning: ply.h must be included after tracer related headers
#include <ply.h>

#ifndef MAX
#define MAX(a, b) ((a > b) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a, b) ((a < b) ? (a) : (b))
#endif

namespace apps {
namespace render {
namespace mpi {
namespace commandline {

using namespace gvt::render::unit;

void PrintUsage(const char *argv) {
  printf("Usage : %s [options]\n", argv);
  printf("  -h, --help\n");
  printf(
      "  -i, --infile <infile> (default: ../data/geom/bunny.obj for obj and "
      "./EnzoPlyData/Enzo8 for ply)\n");
  printf("  -a, --adapter <embree | manta | optix> (default: embree)\n");
  printf("  -t, --tracer <0-3> (default: 0)\n");
  printf(
      "      0: ASYNC_DOMAIN, 1: ASYNC_IMAGE, 2: SYNC_DOMAIN, 3: SYNC_IMAGE, "
      "4: PING_TEST\n");
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
  printf("  -m, --model-name <model_name>\n");
  printf(
      "  --lpos <x, y, z> specify point light position. (512.0, "
      "512.0, 2048.0)\n");
  printf(
      "  --lcolor <x, y, z> specify point light color (default: 100.0, "
      "100.0, 500.0).\n");
  printf(
      "  --eye <x, y, z> specify camera position. (default: 512.0 512.0 "
      "4096.0)\n");
  printf(
      "  --look <x, y, z> specify lookat position (default: 512.0 512.0 "
      "0.0).\n");
  printf("  --up <x, y, z> specify up vector (default: 0 1 0).\n");
  printf("  --fov <degree> specify field of view in degree (default: 25).\n");
  printf("  --ray-max-depth <value> specify ray max. depth (default: 1).\n");
  printf("  --ray-samples <value> specify number of samples (default: 1).\n");
}

void Parse(int argc, char **argv, Options *options) {
  MpiInfo mpi;
  // MPI_Comm_rank(MPI_COMM_WORLD, &mpi.rank);
  // MPI_Comm_size(MPI_COMM_WORLD, &mpi.size);

  // default settings
  options->tracer = Options::ASYNC_DOMAIN;
  options->adapter = Options::EMBREE;
  options->width = 1920;
  options->height = 1080;
  options->obj = false;
  options->instanceCountX = 1;
  options->instanceCountY = 1;
  options->instanceCountZ = 1;
  options->numFrames = 1;
  // options->numTbbThreads;
  // options->infile;
  options->model_name = std::string("model");

  // light
  options->light_position = glm::vec3(512.0, 512.0, 2048.0);
  options->light_color = glm::vec3(100.0, 100.0, 500.0);

  // camera
  options->eye = glm::vec3(512.0, 512.0, 4096.0);
  options->look = glm::vec3(512.0, 512.0, 0.0);
  options->up = glm::vec3(0.0, 1.0, 0.0);
  options->fov = 25.0;

  // ray
  options->ray_max_depth = 1;
  options->ray_samples = 1;

  options->numTbbThreads = -1;
  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
      PrintUsage(argv[0]);
      exit(0);
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
    } else if (strcmp(argv[i], "-m") == 0 ||
               strcmp(argv[i], "--model-name") == 0) {
      options->model_name = argv[++i];
    } else if (strcmp(argv[i], "--eye") == 0) {
      options->eye[0] = atof(argv[++i]);
      options->eye[1] = atof(argv[++i]);
      options->eye[2] = atof(argv[++i]);
    } else if (strcmp(argv[i], "--look") == 0) {
      options->look[0] = atof(argv[++i]);
      options->look[1] = atof(argv[++i]);
      options->look[2] = atof(argv[++i]);
    } else if (strcmp(argv[i], "--up") == 0) {
      options->up[0] = atof(argv[++i]);
      options->up[1] = atof(argv[++i]);
      options->up[2] = atof(argv[++i]);
    } else if (strcmp(argv[i], "--fov") == 0) {
      options->fov = atof(argv[++i]);
    } else if (strcmp(argv[i], "--lpos") == 0) {
      options->light_position[0] = atof(argv[++i]);
      options->light_position[1] = atof(argv[++i]);
      options->light_position[2] = atof(argv[++i]);
    } else if (strcmp(argv[i], "--lcolor") == 0) {
      options->light_color[0] = atof(argv[++i]);
      options->light_color[1] = atof(argv[++i]);
      options->light_color[2] = atof(argv[++i]);
    } else {
      printf("error: %s not defined\n", argv[i]);
      exit(1);
    }
  }
  if (options->numTbbThreads <= 0) {
    if (options->tracer == Options::ASYNC_DOMAIN ||
        options->tracer == Options::ASYNC_IMAGE) {
      options->numTbbThreads = MAX(1, std::thread::hardware_concurrency() - 2);
    } else {
      options->numTbbThreads = MAX(1, std::thread::hardware_concurrency());
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

}  // namespace commandline
}  // namespace mpi
}  // namespace render
}  // namespace apps

namespace apps {
namespace render {
namespace mpi {

using namespace gvt::render::unit;
using namespace gvt::core::time;
using namespace gvt::render::data::primitives;
using namespace gvt::render::data::scene;
using namespace gvt::render::schedule;
using namespace gvt::render::unit::profiler;

// global variables
gvt::render::data::scene::gvtPerspectiveCamera *g_camera = NULL;
gvt::render::data::scene::Image *g_image = NULL;

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
    {"x", Float32, Float32, offsetof(Vertex, x), 0, 0, 0, 0},
    {"y", Float32, Float32, offsetof(Vertex, y), 0, 0, 0, 0},
    {"z", Float32, Float32, offsetof(Vertex, z), 0, 0, 0, 0},
    {"nx", Float32, Float32, offsetof(Vertex, nx), 0, 0, 0, 0},
    {"ny", Float32, Float32, offsetof(Vertex, ny), 0, 0, 0, 0},
    {"nz", Float32, Float32, offsetof(Vertex, nz), 0, 0, 0, 0},
};

PlyProperty face_props[] = {
    /* list of property information for a face */
    {"vertex_indices", Int32, Int32, offsetof(Face, verts), 1, Uint8, Uint8,
     offsetof(Face, nverts)},
};
static Vertex **vlist;
static Face **flist;

template <class T>
void DeallocatePly(std::map<T **, std::queue<T *>> &ply_mem_map) {
  for (auto &m : ply_mem_map) {
    T **pp = m.first;
    std::queue<T *> &q = m.second;
    while (!q.empty()) {
      T *p = q.front();
      q.pop();
      free(p);
    }
    free(pp);
  }
}

bool FileExists(const char *path) {
  struct stat buf;
  return (stat(path, &buf) == 0);
}

bool IsDir(const char *path) {
  struct stat buf;
  stat(path, &buf);
  return S_ISDIR(buf.st_mode);
}

std::vector<std::string> FindPly(const std::string dirname) {
  glob_t result;
  std::string exp = dirname + "/*.ply";
  glob(exp.c_str(), GLOB_TILDE, NULL, &result);
  std::vector<std::string> ret;
  for (int i = 0; i < result.gl_pathc; i++) {
    ret.push_back(std::string(result.gl_pathv[i]));
  }
  globfree(&result);
  return ret;
}

void CreateDatabase(const MpiInfo &mpi, const commandline::Options &options) {
  // mess I use to open and read the ply file with the c utils I found.
  PlyFile *in_ply;
  Vertex *vert;
  Face *face;
  int elem_count, nfaces, nverts;
  int i, j, k;
  float xmin, ymin, zmin, xmax, ymax, zmax;
  char *elem_name;
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

  std::map<Vertex **, std::queue<Vertex *>> vlists_map;
  std::map<Face **, std::queue<Face *>> flists_map;

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

  if (!FileExists(rootdir.c_str())) {
    std::cout << "File \"" << rootdir << "\" does not exist. Exiting."
              << std::endl;
    exit(1);
  }

  if (!IsDir(rootdir.c_str())) {
    std::cout << "File \"" << rootdir << "\" is not a directory. Exiting."
              << std::endl;
    exit(1);
  }

  std::vector<std::string> files = FindPly(rootdir);

  if (files.empty()) {
    std::cout << "Directory \"" << rootdir
              << "\" contains no .ply files. Exiting." << std::endl;
    exit(1);
  }

  // read 'em
  std::vector<std::string>::const_iterator file;

  for (file = files.begin(), k = 0; file != files.end(); file++, k++) {
    int owner_process = k % mpi.size;

    // sprintf(txt, "%d", k);
    // filename = "block";
    // filename += txt;
    gvt::core::DBNodeH EnzoMeshNode =
        cntxt->createNodeFromType("Mesh", *file, dataNodes.UUID());
    // read in some ply data and get ready to load it into the mesh
    // filepath = rootdir + "block" + std::string(txt) + ".ply";
    // filepath = rootdir + "/" + filename + ".ply";
    filepath = *file;

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
    // close_ply(in_ply);
    // smoosh data into the mesh object
    {
      Mesh *mesh = NULL;
      if (mpi.rank == owner_process) {
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

        mesh = new Mesh(m);
      }

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

        if (mesh) {
          mesh->addVertex(glm::vec3(vert->x, vert->y, vert->z));
        }
      }
      glm::vec3 lower(xmin, ymin, zmin);
      glm::vec3 upper(xmax, ymax, zmax);
      Box3D *meshbbox = new gvt::render::data::primitives::Box3D(lower, upper);

      EnzoMeshNode["file"] = std::string(filepath);
      EnzoMeshNode["bbox"] = (unsigned long long)meshbbox;

      if (mesh) {
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
        EnzoMeshNode["ptr"] = (unsigned long long)mesh;
      } else {
        EnzoMeshNode["ptr"] = (unsigned long long)0;
      }
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

    // free memory
    DeallocatePly<Vertex>(vlists_map);
    DeallocatePly<Face>(flists_map);

    close_ply(in_ply);
  }

  // add lights, camera, and film to the database
  gvt::core::DBNodeH lightNodes =
      cntxt->createNodeFromType("Lights", "Lights", root.UUID());
  gvt::core::DBNodeH lightNode =
      cntxt->createNodeFromType("PointLight", "conelight", lightNodes.UUID());
  lightNode["position"] = options.light_position;
  lightNode["color"] = options.light_color;
  // camera
  gvt::core::DBNodeH camNode =
      cntxt->createNodeFromType("Camera", "conecam", root.UUID());
  camNode["eyePoint"] = options.eye;
  camNode["focus"] = options.look;
  camNode["upVector"] = options.up;
  camNode["fov"] = static_cast<float>(options.fov * M_PI / 180.0);
  camNode["rayMaxDepth"] = static_cast<int>(options.ray_max_depth);
  camNode["raySamples"] = static_cast<int>(options.ray_samples);
  // film
  gvt::core::DBNodeH filmNode =
      cntxt->createNodeFromType("Film", "conefilm", root.UUID());
  filmNode["width"] = options.width;
  filmNode["height"] = options.height;

  // if (cmd.isSet("eye")) {
  //  std::vector<float> eye = cmd.getValue<float>("eye");
  //  camNode["eyePoint"] = glm::vec3(eye[0], eye[1], eye[2]);
  //}

  // if (cmd.isSet("look")) {
  //  std::vector<float> eye = cmd.getValue<float>("look");
  //  camNode["focus"] = glm::vec3(eye[0], eye[1], eye[2]);
  //}
  // if (cmd.isSet("wsize")) {
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
  if (options.tracer == commandline::Options::ASYNC_DOMAIN ||
      options.tracer == commandline::Options::SYNC_DOMAIN) {
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
  if (options.adapter == commandline::Options::EMBREE) {
    schedNode["adapter"] = gvt::render::adapter::Embree;
  } else if (options.adapter == commandline::Options::MANTA) {
    schedNode["adapter"] = gvt::render::adapter::Manta;
  } else if (options.adapter == commandline::Options::OPTIX) {
    schedNode["adapter"] = gvt::render::adapter::Optix;
  } else {
    schedNode["adapter"] = gvt::render::adapter::Embree;
  }

  // end db setup

  // use db to create structs needed by system

  // setup gvtCamera from database entries
  g_camera = new gvt::render::data::scene::gvtPerspectiveCamera;
  glm::vec3 cameraposition = camNode["eyePoint"].value().tovec3();
  glm::vec3 focus = camNode["focus"].value().tovec3();
  float fov = camNode["fov"].value().toFloat();
  glm::vec3 up = camNode["upVector"].value().tovec3();
  int rayMaxDepth = camNode["rayMaxDepth"].value().toInteger();
  int raySamples = camNode["raySamples"].value().toInteger();
  g_camera->lookAt(cameraposition, focus, up);
  g_camera->setMaxDepth(rayMaxDepth);
  g_camera->setSamples(raySamples);
  g_camera->setFOV(fov);
  g_camera->setFilmsize(filmNode["width"].value().toInteger(),
                        filmNode["height"].value().toInteger());

}  // void CreateDatabase(const commandline::Options& options) {

void Render(int argc, char **argv) {
  MpiInfo mpi;
  MPI_Comm_rank(MPI_COMM_WORLD, &mpi.rank);
  MPI_Comm_size(MPI_COMM_WORLD, &mpi.size);

  timer t_database(false, "database timer:");

  if (mpi.rank == 0) std::cout << "mpi size: " << mpi.size << std::endl;

  commandline::Options options;
  commandline::Parse(argc, argv, &options);

  // initialize tbb
  tbb::task_scheduler_init init(options.numTbbThreads);

  switch (options.tracer) {
    case commandline::Options::PING_TEST: {
      if (mpi.rank == 0) std::cout << "start PING_TEST" << std::endl;

      Worker worker(mpi, options, NULL, NULL);
      // mpi = worker.GetMpiInfo();
      // worker.InitTracer(options, NULL, NULL);

      worker.Render();
      worker.Wait();
    } break;

    case commandline::Options::ASYNC_DOMAIN: {
      if (mpi.rank == 0) std::cout << "start ASYNC_DOMAIN" << std::endl;

      std::cout << "rank " << mpi.rank << " creating database." << std::endl;
      t_database.start();
      CreateDatabase(mpi, options);
      t_database.stop();
      std::cout << "rank " << mpi.rank << " done creating database." << std::endl;

      g_image = new Image(g_camera->getFilmSizeWidth(),
                          g_camera->getFilmSizeHeight(), "mpi");

      Worker worker(mpi, options, g_camera, g_image);

      // mpi = worker.GetMpiInfo();

      // worker.InitTracer(options, g_camera, g_image);

      DomainTracer *domain_tracer =
          static_cast<DomainTracer *>(worker.GetTracer());
      Profiler &profiler = domain_tracer->GetProfiler();

// warm up
#ifndef NDEBUG
      std::cout << "rank " << mpi.rank << " start warming up\n\n";
#endif
      for (int i = 0; i < 10; i++) {
        g_camera->AllocateCameraRays();
        g_camera->generateRays();
        g_image->clear();
        worker.Render();
#ifndef NDEBUG
        std::cout << "rank " << mpi.rank << " warm up frame " << i
                  << " done\n\n";
#endif
      }

#ifndef NDEBUG
      std::cout << "rank " << mpi.rank << " start active frames" << std::endl;
#endif
      profiler.Reset();
      profiler.Start(Profiler::TOTAL_TIME);

      for (int i = 0; i < 100; i++) {
        profiler.Start(Profiler::CAMERA_RAY);
        g_camera->AllocateCameraRays();
        g_camera->generateRays();
        g_image->clear();
        profiler.Stop(Profiler::CAMERA_RAY);

        worker.Render();
#ifndef NDEBUG
        std::cout << "rank " << mpi.rank << " active frame " << i
                  << " done\n\n";
#endif
      }

      profiler.Stop(Profiler::TOTAL_TIME);

      if (mpi.rank == 0) {
        g_image->Write();
        worker.Quit();
      }
      worker.Wait();

      std::ostringstream rank_ss;
      rank_ss << mpi.rank;
      std::string rank_str = rank_ss.str();

      std::ostringstream mpi_size_ss;
      mpi_size_ss << mpi.size;
      std::string mpi_size_str = mpi_size_ss.str();

      std::string filename("prof_" + options.model_name +
                           "_async_domain_size_" + mpi_size_str + "_rank_" +
                           rank_str + ".txt");

      profiler.WriteToFile(filename, mpi.rank);

    } break;

    case commandline::Options::SYNC_DOMAIN: {
      // MPI_Init(&argc, &argv);
      MPI_Comm_rank(MPI_COMM_WORLD, &mpi.rank);
      MPI_Comm_size(MPI_COMM_WORLD, &mpi.size);

      if (mpi.rank == 0) std::cout << "start SYNC_DOMAIN" << std::endl;

      std::cout << "rank " << mpi.rank << " creating database." << std::endl;
      t_database.start();
      CreateDatabase(mpi, options);
      t_database.stop();
      std::cout << "rank " << mpi.rank << " done creating database." << std::endl;

      g_image = new Image(g_camera->getFilmSizeWidth(),
                          g_camera->getFilmSizeHeight(), "mpi");

      g_camera->AllocateCameraRays();
      g_camera->generateRays();

      gvt::render::algorithm::Tracer<DomainScheduler> tracer(g_camera->rays,
                                                             *g_image);
      Profiler &profiler = tracer.GetProfiler();

      for (int z = 0; z < 10; z++) {
        g_camera->AllocateCameraRays();
        g_camera->generateRays();
        g_image->clear();
        tracer();
      }

      profiler.Start(Profiler::TOTAL_TIME);
      for (int z = 0; z < 100; z++) {
        profiler.Start(Profiler::CAMERA_RAY);
        g_camera->AllocateCameraRays();
        g_camera->generateRays();
        g_image->clear();
        profiler.Stop(Profiler::CAMERA_RAY);

        tracer();
      }
      profiler.Stop(Profiler::TOTAL_TIME);

      g_image->Write();

      std::ostringstream rank_ss;
      rank_ss << mpi.rank;
      std::string rank_str = rank_ss.str();

      std::ostringstream mpi_size_ss;
      mpi_size_ss << mpi.size;
      std::string mpi_size_str = mpi_size_ss.str();

      std::string filename("prof_" + options.model_name + "_sync_domain_size_" +
                           mpi_size_str + "_rank_" + rank_str + ".txt");

      profiler.WriteToFile(filename, mpi.rank);

    } break;

    default: {
      std::cout << "rank " << mpi.rank
                << " error found unsupported tracer type " << options.tracer
                << std::endl;
      exit(1);
    } break;
  }
  // clean up
  if (g_image) delete g_image;
  if (g_camera) delete g_camera;
}

}  // namespace mpi
}  // namespace render
}  // namespace apps

int main(int argc, char **argv) {
  // MPI_Init(&argc, &argv);
  int pvd;
  MPI_Init_thread(&argc, &argv, MPI_THREAD_MULTIPLE, &pvd);
  if ((pvd != MPI_THREAD_MULTIPLE)) {
    std::cerr << "error: mpi_thread_multiple not available\n";
    exit(1);
  }

  apps::render::mpi::Render(argc, argv);

  MPI_Finalize();
}

