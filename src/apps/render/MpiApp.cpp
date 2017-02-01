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
//
// MpiApp.cpp
//

#include <cmath>
#include <cstdlib>
#include <glm/glm.hpp>
#include <glm/gtx/rotate_vector.hpp>
#include <glob.h>
#include <iostream>
#include <map>
#include <mpi.h>
#include <sstream>
#include <sys/stat.h>
#include <tbb/task_scheduler_init.h>
#include <thread>

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
// #include "gvt/render/unit/RayTracer.h"
#include "gvt/render/unit/CommonWorks.h"
#include "gvt/render/unit/DomainTracer.h"
#include "gvt/render/unit/DomainWorks.h"
#include "gvt/render/unit/Profiler.h"
#include "gvt/render/unit/TestTracer.h"
#include "gvt/render/unit/Worker.h"

// sync schedule
#include "gvt/render/algorithm/DomainTracer.h"
#include "gvt/render/algorithm/ImageTracer.h"

#include "apps/render/MpiApp.h"

#include "gvt/render/data/reader/ObjReader.h"

// warning: ply.h must be included after tracer related headers
#include <ply.h>

#if defined(__APPLE__)
#include <GLUT/glut.h>
#else
#include <GL/freeglut.h>
#endif

#ifndef MAX
#define MAX(a, b) ((a > b) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a, b) ((a < b) ? (a) : (b))
#endif

// #define PRINT_FRAME_NUMBER

namespace apps {
namespace render {
namespace mpi {
namespace commandline {

using namespace gvt::render::unit;

void PrintUsage(const char *argv) {
  printf("Usage : %s [options]\n", argv);
  printf("  -h, --help\n");
  printf("  -i, --infile <infile> (default: ../data/geom/bunny.obj for obj and "
         "./EnzoPlyData/Enzo8 for ply)\n");
  printf("  -a, --adapter <embree | manta | optix> (default: embree)\n");
  printf("  -t, --tracer <0-3> (default: 0)\n");
  printf("      0: ASYNC_DOMAIN, 1: ASYNC_IMAGE, 2: SYNC_DOMAIN, 3: SYNC_IMAGE, "
         "4: PING_TEST\n");
  printf("  -n, --num-instances <x, y, z> specify the number of instances in each "
         "direction (default: 1 1 1). "
         "effective only with obj.\n");
  printf("  --obj use obj models (not ply models)\n");
  printf("  -W, --width <image_width> (default: 1920)\n");
  printf("  -H, --height <image_height> (default: 1080)\n");
  printf("  -N, --num-frames <num_frames> (default: 1)\n");
  printf("  --tbb <num_tbb_threads>\n");
  printf("      (default: # cores for sync. schedulers or # cores - 2 for async. "
         "schedulers)\n");
  printf("  -m, --model-name <model_name>\n");
  printf("  --lpos <x, y, z> specify point light position. (512.0, "
         "512.0, 2048.0)\n");
  printf("  --lcolor <x, y, z> specify point light color (default: 100.0, "
         "100.0, 500.0).\n");
  printf("  --point-light <x, y, z, r, g, b> ad point light source.\n");
  printf("  --eye <x, y, z> specify camera position. (default: 512.0 512.0 "
         "4096.0)\n");
  printf("  --look <x, y, z> specify lookat position (default: 512.0 512.0 "
         "0.0).\n");
  printf("  --up <x, y, z> specify up vector (default: 0 1 0).\n");
  printf("  --fov <degree> specify field of view in degree (default: 25).\n");
  printf("  --ray-depth <value> specify ray max. depth (default: 1).\n");
  printf("  --ray-samples <value> specify number of samples (default: 1).\n");
  printf("  --warmup <value> specify number of warm-up frames (default: 10).\n");
  printf("  --active <value> specify number of active frames (default: 100).\n");
  printf("  --gl enable interactive mode.\n");
  printf("  --ply-with-color PLY files having color information.\n");
  printf("  --shading-model <0: lambert, 1: phong, 2: blinn>.\n");
}

void Parse(int argc, char **argv, Options *options) {
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
    } else if (strcmp(argv[i], "-a") == 0 || strcmp(argv[i], "--adapter") == 0) {
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
    } else if (strcmp(argv[i], "-n") == 0 || strcmp(argv[i], "--num-instances") == 0) {
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
    } else if (strcmp(argv[i], "-N") == 0 || strcmp(argv[i], "--num-frames") == 0) {
      options->numFrames = atoi(argv[++i]);
    } else if (strcmp(argv[i], "--tbb") == 0) {
      options->numTbbThreads = atoi(argv[++i]);
    } else if (strcmp(argv[i], "-m") == 0 || strcmp(argv[i], "--model-name") == 0) {
      options->model_name = argv[++i];
    } else if (strcmp(argv[i], "--eye") == 0) {
      options->eye[0] = atof(argv[++i]);
      options->eye[1] = atof(argv[++i]);
      options->eye[2] = atof(argv[++i]);
      options->set_eye = true;
    } else if (strcmp(argv[i], "--look") == 0) {
      options->look[0] = atof(argv[++i]);
      options->look[1] = atof(argv[++i]);
      options->look[2] = atof(argv[++i]);
      options->set_look = true;
    } else if (strcmp(argv[i], "--up") == 0) {
      options->up[0] = atof(argv[++i]);
      options->up[1] = atof(argv[++i]);
      options->up[2] = atof(argv[++i]);
      options->set_up = true;
    } else if (strcmp(argv[i], "--fov") == 0) {
      options->fov = atof(argv[++i]);
    } else if (strcmp(argv[i], "--lpos") == 0) {
      options->light_position[0] = atof(argv[++i]);
      options->light_position[1] = atof(argv[++i]);
      options->light_position[2] = atof(argv[++i]);
      options->set_light_position = true;
    } else if (strcmp(argv[i], "--lcolor") == 0) {
      options->light_color[0] = atof(argv[++i]);
      options->light_color[1] = atof(argv[++i]);
      options->light_color[2] = atof(argv[++i]);
      options->set_light_color = true;
    } else if (strcmp(argv[i], "--point-light") == 0) {
      PointLightInfo info;
      for (int j = 0; j < 3; ++j) {
        info.position[j] = atof(argv[++i]);
      }
      for (int j = 0; j < 3; ++j) {
        info.color[j] = atof(argv[++i]);
      }
      options->point_lights.push_back(info);
    } else if (strcmp(argv[i], "--ray-depth") == 0) {
      options->ray_depth = atoi(argv[++i]);
    } else if (strcmp(argv[i], "--ray-samples") == 0) {
      options->ray_samples = atoi(argv[++i]);
    } else if (strcmp(argv[i], "--warmup") == 0) {
      options->warmup_frames = atoi(argv[++i]);
    } else if (strcmp(argv[i], "--active") == 0) {
      options->active_frames = atoi(argv[++i]);
    } else if (strcmp(argv[i], "--gl") == 0) {
      options->interactive = true;
    } else if (strcmp(argv[i], "--ply-with-color") == 0) {
      options->ply_with_color = true;
    } else if (strcmp(argv[i], "--shading-model") == 0) {
      options->shading_model = atoi(argv[++i]);
    } else {
      printf("error: %s not defined\n", argv[i]);
      exit(1);
    }
  }
  if (options->numTbbThreads <= 0) {
    if (options->tracer == Options::ASYNC_DOMAIN || options->tracer == Options::ASYNC_IMAGE) {
      options->numTbbThreads = MAX(1, std::thread::hardware_concurrency() - 1);
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

} // namespace commandline
} // namespace mpi
} // namespace render
} // namespace apps

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

apps::render::mpi::commandline::Options *g_options = NULL;
gvt::render::data::primitives::Box3D g_scene_bound;

// global variables
// gvt::render::unit::RayTracer* tracer;
bool g_quit_key_entered = false;
int g_window;
// std::size_t s_num_frames = 0;
int g_num_warmup_frames;
int g_num_active_frames;
int g_width;
int g_height;
GLubyte *imagebuffer;
int g_tracer_mode;
int g_mpiRank;
int g_mpiSize;
gvt::render::unit::Worker *g_worker;
gvt::render::algorithm::AbstractTrace *g_tracer;
int g_num_frames = 0;

class CameraState {
public:
  CameraState() : sensitivity(1.f) {}

  void increaseSensitivity() { sensitivity *= 4.f; }
  void decreaseSensitivity() { sensitivity /= 4.f; }
  float getSensitivity() { return sensitivity; }

  void reset(const commandline::Options &options, const Box3D &scene_bound) {
    bound = scene_bound;
    glm::vec3 eye, look;
    if (options.set_eye) {
      eye = options.eye;
    } else {
      float diag = glm::length(scene_bound.extent());
      eye = scene_bound.centroid() + (2.f * diag * glm::vec3(0.f, 0.f, 1.f));
    }
    look = options.set_look ? options.look : scene_bound.centroid();
    // *up = options.set_up ? options.up : glm::vec3(0.0, 1.0, 0.0);
    pos = eye;
    center = look;
    glm::vec3 camvec = eye - look;
    rho = glm::length(camvec);
    theta = glm::atan(camvec.y, camvec.z);
    camvec = glm::normalize(camvec);
    phi = glm::acos(camvec.y);
    float sin_phi = sin(phi);
    if (sin_phi == 0) {
      phi += 0.00001f;
      sin_phi = sin(phi);
    }
    up = glm::vec3(0, sin_phi > 0 ? 1 : -1, 0);
  }

  void zoom(float offset) {
    float extent = glm::length(bound.extent());
    float new_rho = glm::clamp(rho - (offset * sensitivity), extent * 0.01f, extent * 10.f);
    pos = (pos - center) * (new_rho / rho) + center;
    rho = new_rho;
  }

  void rotate(float dx, float dy, int image_width, int image_height) {
    float dtheta = dx * (M_PI / image_width);
    float dphi = dy * (M_PI / image_height);
    theta += dtheta;
    phi = glm::clamp(phi + dphi, (float)0.0, (float)M_PI);
    float sin_phi = sin(phi);
    if (sin_phi == 0) {
      phi += 0.00001f;
      sin_phi = sin(phi);
    }
    glm::vec3 camvec(rho * sin_phi * sin(theta), rho * cos(phi), rho * sin_phi * cos(theta));
    pos = center + camvec;
    up = glm::vec3(0, sin_phi > 0 ? 1 : -1, 0);
  }

  void pan(float dx, float dy, const glm::mat4 &c2w) {
    float sdx = dx * 0.001f * sensitivity;
    float sdy = -dy * 0.001f * sensitivity;
    glm::vec4 disp = (c2w[0] * sdx) + (c2w[1] * sdy);
    pos += glm::vec3(disp);
    center += glm::vec3(disp);
  }

  glm::vec3 getPos() const { return pos; }
  glm::vec3 getUp() const { return up; }
  glm::vec3 getCenter() const { return center; }

private:
  float rho, phi, theta;
  glm::vec3 pos, up, center;
  Box3D bound;
  float sensitivity;
};

CameraState g_camera_state;

typedef struct Vertex {
  float x, y, z;
  unsigned char cx, cy, cz;
  // float nx, ny, nz;
  void *other_props; /* other properties */
} Vertex;

typedef struct Face {
  unsigned char nverts; /* number of vertex indices in list */
  int *verts;           /* vertex index list */
  void *other_props;    /* other properties */
} Face;

PlyProperty vert_props[] = {
  /* list of property information for a vertex */
  { "x", Float32, Float32, offsetof(Vertex, x), PLY_SCALAR, 0, 0, 0 },
  { "y", Float32, Float32, offsetof(Vertex, y), PLY_SCALAR, 0, 0, 0 },
  { "z", Float32, Float32, offsetof(Vertex, z), PLY_SCALAR, 0, 0, 0 },
  { "red", Uint8, Uint8, offsetof(Vertex, cx), PLY_SCALAR, 0, 0, 0 },
  { "green", Uint8, Uint8, offsetof(Vertex, cy), PLY_SCALAR, 0, 0, 0 },
  { "blue", Uint8, Uint8, offsetof(Vertex, cz), PLY_SCALAR, 0, 0, 0 }
  // { "nx", Float32, Float32, offsetof(Vertex, nx), 0, 0, 0, 0 },
  // { "ny", Float32, Float32, offsetof(Vertex, ny), 0, 0, 0, 0 },
  // { "nz", Float32, Float32, offsetof(Vertex, nz), 0, 0, 0, 0 },
};

PlyProperty face_props[] = {
  /* list of property information for a face */
  { "vertex_indices", Int32, Int32, offsetof(Face, verts), 1, Uint8, Uint8, offsetof(Face, nverts) },
};
static Vertex **vlist;
static Face **flist;

template <class T> void DeallocatePly(std::map<T **, std::queue<T *> > &ply_mem_map) {
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

std::string GetTestName(const MpiInfo &mpi, const commandline::Options &options) {
  std::string tracer_name;
  if (options.tracer == commandline::Options::ASYNC_DOMAIN) {
    tracer_name = "async_domain";
  } else if (options.tracer == commandline::Options::SYNC_DOMAIN) {
    tracer_name = "sync_domain";
  } else if (options.tracer == commandline::Options::SYNC_IMAGE) {
    tracer_name = "sync_image";
  } else {
    tracer_name = "unknown_tracer";
  }

  std::ostringstream rank_ss;
  rank_ss << mpi.rank;
  std::string rank_str = rank_ss.str();

  std::ostringstream mpi_size_ss;
  mpi_size_ss << mpi.size;
  std::string mpi_size_str = mpi_size_ss.str();

  std::string filename("prof_" + options.model_name + "_" + tracer_name + "_size_" + mpi_size_str + "_rank_" +
                       rank_str);
  return filename;
}

void initCamera(const apps::render::mpi::commandline::Options &options,
                const gvt::render::data::primitives::Box3D &scene_bound) {
  gvt::render::RenderContext *cntxt = gvt::render::RenderContext::instance();
  gvt::core::DBNodeH root = cntxt->getRootNode();
  gvt::core::DBNodeH cam_node = cntxt->createNodeFromType("Camera", "cam", root.UUID());

  g_camera_state.reset(options, scene_bound);

  int ray_max_depth = static_cast<int>(options.ray_depth);
  int ray_samples = static_cast<int>(options.ray_samples);
  float fov = (float)(45.0 * M_PI / 180.0);

  cam_node["eyePoint"] = g_camera_state.getPos();
  cam_node["focus"] = g_camera_state.getCenter();
  cam_node["upVector"] = g_camera_state.getUp();
  cam_node["fov"] = fov;
  cam_node["rayMaxDepth"] = ray_max_depth;
  cam_node["raySamples"] = ray_samples;
  cam_node["jitterWindowSize"] = (float)0;

  g_camera = new gvt::render::data::scene::gvtPerspectiveCamera;

  g_camera->lookAt(g_camera_state.getPos(), g_camera_state.getCenter(), g_camera_state.getUp());
  g_camera->setMaxDepth(ray_max_depth);
  g_camera->setSamples(ray_samples);
  g_camera->setFOV(fov);
  g_camera->setFilmsize(options.width, options.height);
}

void initLights(const apps::render::mpi::commandline::Options &options,
                const gvt::render::data::primitives::Box3D &scene_bound) {

  gvt::render::RenderContext *cntxt = gvt::render::RenderContext::instance();
  gvt::core::DBNodeH root = cntxt->getRootNode();

  // add point light sources
  gvt::core::DBNodeH lightNodes = cntxt->createNodeFromType("Lights", "Lights", root.UUID());

  // gvt::core::DBNodeH lightNode = cntxt->createNodeFromType("PointLight", "light", lightNodes.UUID());
  // lightNode["position"] = glm::vec3(0.0, 0.1, 0.5);
  // lightNode["color"] = glm::vec3(1.0, 1.0, 1.0);

  gvt::core::DBNodeH point_light;
  for (std::size_t i = 0; i < options.point_lights.size(); ++i) {
    std::stringstream ss;
    ss << i;
    std::string name("p");
    name += ss.str();
    point_light = cntxt->createNodeFromType("PointLight", name, lightNodes.UUID());
    point_light["position"] = options.point_lights[i].position;
    point_light["color"] = options.point_lights[i].color;
  }

  // bool light_specified = options.set_light_position || options.set_light_color;
  // if (light_specified || (!light_specified && options.point_lights.empty())) {
  //   std::stringstream ss;
  //   ss << options.point_lights.size();
  //   std::string name("p");
  //   name += ss.str();
  //   point_light = cntxt->createNodeFromType("PointLight", name, lightNodes.UUID());
  //   point_light["position"] = options.light_position;
  //   point_light["color"] = options.light_color;
  // }

  // the user did not specify any light source
  // use default light sources
  if (options.point_lights.empty()) {
    glm::vec3 centroid = scene_bound.centroid();
    glm::vec3 p1 = centroid + 2.0f * (scene_bound.bounds_min - centroid);
    glm::vec3 p2 = centroid + 2.0f * (scene_bound.bounds_max - centroid);
    glm::vec3 p3 =
        centroid +
        2.0f * (glm::vec3(scene_bound.bounds_min[0], scene_bound.bounds_max[1], scene_bound.bounds_min[2]) - centroid);
    glm::vec3 p4 =
        centroid +
        2.0f * (glm::vec3(scene_bound.bounds_max[0], scene_bound.bounds_min[1], scene_bound.bounds_max[2]) - centroid);

    glm::vec3 p5 =
        centroid +
        2.0f * (glm::vec3(scene_bound.bounds_min[0], scene_bound.bounds_max[1], scene_bound.bounds_max[2]) - centroid);
    glm::vec3 p6 =
        centroid +
        2.0f * (glm::vec3(scene_bound.bounds_max[0], scene_bound.bounds_max[1], scene_bound.bounds_min[2]) - centroid);

    point_light = cntxt->createNodeFromType("PointLight", "p1", lightNodes.UUID());
    point_light["position"] = p1;
    point_light["color"] = options.set_light_color ? options.light_color : glm::vec3(0.5);

    point_light = cntxt->createNodeFromType("PointLight", "p2", lightNodes.UUID());
    point_light["position"] = p2;
    point_light["color"] = options.set_light_color ? options.light_color : glm::vec3(0.5);

    point_light = cntxt->createNodeFromType("PointLight", "p3", lightNodes.UUID());
    point_light["position"] = p3;
    point_light["color"] = options.set_light_color ? options.light_color : glm::vec3(0.5);

    point_light = cntxt->createNodeFromType("PointLight", "p4", lightNodes.UUID());
    point_light["position"] = p4;
    point_light["color"] = options.set_light_color ? options.light_color : glm::vec3(0.5);

    point_light = cntxt->createNodeFromType("PointLight", "p5", lightNodes.UUID());
    point_light["position"] = p5;
    point_light["color"] = options.set_light_color ? options.light_color : glm::vec3(0.5);

    point_light = cntxt->createNodeFromType("PointLight", "p6", lightNodes.UUID());
    point_light["position"] = p6;
    point_light["color"] = options.set_light_color ? options.light_color : glm::vec3(0.5);
  }

  // TODO: ambient light is currently not supported
  // gvt::core::DBNodeH ambient_light = cntxt->createNodeFromType("AmbientLight", "p5", lightNodes.UUID());
  // ambient_light["color"] = options.set_light_color ? options.light_color : glm::vec3(1.0);
}

void CreatePlyDatabase(const MpiInfo &mpi, const commandline::Options &options) {
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

  std::map<Vertex **, std::queue<Vertex *> > vlists_map;
  std::map<Face **, std::queue<Face *> > flists_map;

  gvt::render::RenderContext *cntxt = gvt::render::RenderContext::instance();
  if (cntxt == NULL) {
    std::cout << "Something went wrong initializing the context" << std::endl;
    exit(0);
  }
  gvt::core::DBNodeH root = cntxt->getRootNode();
  gvt::core::DBNodeH dataNodes = cntxt->createNodeFromType("Data", "Data", root.UUID());
  gvt::core::DBNodeH instNodes = cntxt->createNodeFromType("Instances", "Instances", root.UUID());

  if (!FileExists(rootdir.c_str())) {
    std::cout << "File \"" << rootdir << "\" does not exist. Exiting." << std::endl;
    exit(1);
  }

  if (!IsDir(rootdir.c_str())) {
    std::cout << "File \"" << rootdir << "\" is not a directory. Exiting." << std::endl;
    exit(1);
  }

  std::vector<std::string> files = FindPly(rootdir);

  if (files.empty()) {
    std::cout << "Directory \"" << rootdir << "\" contains no .ply files. Exiting." << std::endl;
    exit(1);
  }

  // read 'em
  std::vector<std::string>::const_iterator file;

  for (file = files.begin(), k = 0; file != files.end(); file++, k++) {
    int instanceId = k;
    // WARNING (hpark): this data distribution must be consistent with the instance-data mapper in the tracer.
    // TODO duplicating the same scheme for now.
    int ownerProcess = mpi.rank;
    if (options.tracer == commandline::Options::ASYNC_DOMAIN || options.tracer == commandline::Options::SYNC_DOMAIN) {
      ownerProcess = instanceId % mpi.size;
    }

    // sprintf(txt, "%d", k);
    // filename = "block";
    // filename += txt;
    gvt::core::DBNodeH EnzoMeshNode = cntxt->createNodeFromType("Mesh", *file, dataNodes.UUID());
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
        if (options.ply_with_color) {
          setup_property_ply(in_ply, &vert_props[3]);
          setup_property_ply(in_ply, &vert_props[4]);
          setup_property_ply(in_ply, &vert_props[5]);
        }
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
      Mesh *mesh = nullptr;
      // load mesh data only if this is the owner process.
      if (mpi.rank == ownerProcess) {
        if (options.ply_with_color) {
          // material to be created inside the mesh adapter
          // color to be retreived directly from mesh
          mesh = new Mesh(nullptr /* Material */);
        } else {
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
          if (options.ply_with_color) {
            mesh->addVertexColor(glm::vec3(vert->cx / 255.f, vert->cy / 255.f, vert->cz / 255.f));
          }
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
          mesh->addFace(face->verts[0] + 1, face->verts[1] + 1, face->verts[2] + 1);
        }
        mesh->generateNormals();
        // add Enzo mesh to the database
        // EnzoMeshNode["file"] =
        // string("/work/01197/semeraro/maverick/DAVEDATA/EnzoPlyDATA/Block0.ply");
        EnzoMeshNode["ptr"] = (unsigned long long)mesh;

        g_scene_bound.merge(*(mesh->getBoundingBox()));
      } else {
        EnzoMeshNode["ptr"] = (unsigned long long)0;
      }
    }

    // add instance
    gvt::core::DBNodeH instnode = cntxt->createNodeFromType("Instance", "inst", instNodes.UUID());
    gvt::core::DBNodeH meshNode = EnzoMeshNode;
    Box3D *mbox = (Box3D *)meshNode["bbox"].value().toULongLong();
    instnode["id"] = instanceId;
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
  } // for (file = files.begin(), k = 0; file != files.end(); file++, k++) {

  // add lights, camera, and film to the database

  // // add point light sources
  // gvt::core::DBNodeH lightNodes = cntxt->createNodeFromType("Lights", "Lights", root.UUID());

  // gvt::core::DBNodeH point_light;
  // for (std::size_t i = 0; i < options.point_lights.size(); ++i) {
  //   std::stringstream ss;
  //   ss << i;
  //   std::string name("p");
  //   name += ss.str();
  //   point_light = cntxt->createNodeFromType("PointLight", name, lightNodes.UUID());
  //   point_light["position"] = options.point_lights[i].position;
  //   point_light["color"] = options.point_lights[i].color;
  // }

  // bool light_specified = options.set_light_position || options.set_light_color;
  // if (light_specified || (!light_specified && options.point_lights.empty())) {
  //   std::stringstream ss;
  //   ss << options.point_lights.size();
  //   std::string name("p");
  //   name += ss.str();
  //   point_light = cntxt->createNodeFromType("PointLight", name, lightNodes.UUID());
  //   point_light["position"] = options.light_position;
  //   point_light["color"] = options.light_color;
  // }

  // lights
  initLights(options, g_scene_bound);

  // 
  // camera
  initCamera(options, g_scene_bound);

  // gvt::core::DBNodeH camNode = cntxt->createNodeFromType("Camera", "conecam", root.UUID());
  // camNode["eyePoint"] = options.eye;
  // camNode["focus"] = options.look;
  // camNode["upVector"] = options.up;
  // camNode["fov"] = static_cast<float>(options.fov * M_PI / 180.0);
  // camNode["rayMaxDepth"] = static_cast<int>(options.ray_depth);
  // camNode["raySamples"] = static_cast<int>(options.ray_samples);

  // film
  gvt::core::DBNodeH filmNode = cntxt->createNodeFromType("Film", "conefilm", root.UUID());
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

  gvt::core::DBNodeH schedNode = cntxt->createNodeFromType("Schedule", "Enzosched", root.UUID());
  // if (cmd.isSet("domain"))
  //   schedNode["type"] = gvt::render::scheduler::Domain;
  // else
  //   schedNode["type"] = gvt::render::scheduler::Image;
  if (options.tracer == commandline::Options::ASYNC_DOMAIN || options.tracer == commandline::Options::SYNC_DOMAIN) {
    schedNode["type"] = gvt::render::scheduler::Domain;
  } else {
    schedNode["type"] = gvt::render::scheduler::Image;
  }

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

  // // setup gvtCamera from database entries
  // g_camera = new gvt::render::data::scene::gvtPerspectiveCamera;
  // glm::vec3 cameraposition = camNode["eyePoint"].value().tovec3();
  // glm::vec3 focus = camNode["focus"].value().tovec3();
  // float fov = camNode["fov"].value().toFloat();
  // glm::vec3 up = camNode["upVector"].value().tovec3();
  // int rayMaxDepth = camNode["rayMaxDepth"].value().toInteger();
  // int raySamples = camNode["raySamples"].value().toInteger();

  // g_camera->lookAt(cameraposition, focus, up);
  // g_camera->setMaxDepth(rayMaxDepth);
  // g_camera->setSamples(raySamples);
  // g_camera->setFOV(fov);
  // g_camera->setFilmsize(filmNode["width"].value().toInteger(), filmNode["height"].value().toInteger());

} // void CreatePlyDatabase(const commandline::Options& options) {

void CreateObjDatabase(const MpiInfo &mpi, const commandline::Options &options) {

  gvt::render::RenderContext *cntxt = gvt::render::RenderContext::instance();
  if (cntxt == NULL) {
    std::cout << "Something went wrong initializing the context" << std::endl;
    exit(0);
  }

  gvt::core::DBNodeH root = cntxt->getRootNode();

  // add the data - mesh in this case
  gvt::core::DBNodeH dataNodes = cntxt->createNodeFromType("Data", "Data", root.UUID());

  gvt::core::DBNodeH objMeshNode = cntxt->createNodeFromType("Mesh", "objmesh", dataNodes.UUID());
  // {
  // std::string objPath = std::string("../data/geom/bunny.obj");
  std::string objPath = options.infile;
  // if(cmd.isSet("obj"))
  // {
  //   objPath = cmd.getValue<std::string>("obj")[0];
  // }

  // path assumes binary is run as bin/gvtFileApp
  gvt::render::data::domain::reader::ObjReader objReader(objPath, options.shading_model);

  // right now mesh must be converted to gvt format
  Mesh *mesh = objReader.getMesh();
  mesh->generateNormals();

  mesh->computeBoundingBox();
  Box3D *meshbbox = mesh->getBoundingBox();
  g_scene_bound = *meshbbox;

  // add bunny mesh to the database

  objMeshNode["file"] = objPath;
  objMeshNode["bbox"] = (unsigned long long)meshbbox;
  objMeshNode["ptr"] = (unsigned long long)mesh;
  // }

  // create the instance
  gvt::core::DBNodeH instNodes = cntxt->createNodeFromType("Instances", "Instances", root.UUID());

  gvt::core::DBNodeH instnode = cntxt->createNodeFromType("Instance", "inst", instNodes.UUID());
  gvt::core::DBNodeH meshNode = objMeshNode;
  Box3D *mbox = (Box3D *)meshNode["bbox"].value().toULongLong();

  instnode["id"] = 0; // unique id per instance
  instnode["meshRef"] = meshNode.UUID();

  // transform bunny
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
  Box3D *ibox = new gvt::render::data::primitives::Box3D(il, ih);
  instnode["bbox"] = (unsigned long long)ibox;
  instnode["centroid"] = ibox->centroid();

  // add lights, camera, and film to the database
  //
  // lights
  initLights(options, g_scene_bound);

  // 
  // camera
  initCamera(options, g_scene_bound);


  // // add point light sources
  // gvt::core::DBNodeH lightNodes = cntxt->createNodeFromType("Lights", "Lights", root.UUID());

  // // gvt::core::DBNodeH lightNode = cntxt->createNodeFromType("PointLight", "light", lightNodes.UUID());
  // // lightNode["position"] = glm::vec3(0.0, 0.1, 0.5);
  // // lightNode["color"] = glm::vec3(1.0, 1.0, 1.0);

  // gvt::core::DBNodeH point_light;
  // for (std::size_t i = 0; i < options.point_lights.size(); ++i) {
  //   std::stringstream ss;
  //   ss << i;
  //   std::string name("p");
  //   name += ss.str();
  //   point_light = cntxt->createNodeFromType("PointLight", name, lightNodes.UUID());
  //   point_light["position"] = options.point_lights[i].position;
  //   point_light["color"] = options.point_lights[i].color;
  // }

  // // bool light_specified = options.set_light_position || options.set_light_color;
  // // if (light_specified || (!light_specified && options.point_lights.empty())) {
  // //   std::stringstream ss;
  // //   ss << options.point_lights.size();
  // //   std::string name("p");
  // //   name += ss.str();
  // //   point_light = cntxt->createNodeFromType("PointLight", name, lightNodes.UUID());
  // //   point_light["position"] = options.light_position;
  // //   point_light["color"] = options.light_color;
  // // }

  // // the user did not specify any light source
  // // use default light source
  // if (options.point_lights.empty()) {
  //   glm::vec3 centroid = meshbbox->centroid();
  //   glm::vec3 p1 = centroid + 2.0f * (meshbbox->bounds_min - centroid);
  //   glm::vec3 p2 = centroid + 2.0f * (meshbbox->bounds_max - centroid);
  //   glm::vec3 p3 =
  //       centroid +
  //       2.0f * (glm::vec3(meshbbox->bounds_min[0], meshbbox->bounds_max[1], meshbbox->bounds_min[2]) - centroid);
  //   glm::vec3 p4 =
  //       centroid +
  //       2.0f * (glm::vec3(meshbbox->bounds_max[0], meshbbox->bounds_min[1], meshbbox->bounds_max[2]) - centroid);

  //   glm::vec3 p5 =
  //       centroid +
  //       2.0f * (glm::vec3(meshbbox->bounds_min[0], meshbbox->bounds_max[1], meshbbox->bounds_max[2]) - centroid);
  //   glm::vec3 p6 =
  //       centroid +
  //       2.0f * (glm::vec3(meshbbox->bounds_max[0], meshbbox->bounds_max[1], meshbbox->bounds_min[2]) - centroid);

  //   point_light = cntxt->createNodeFromType("PointLight", "p1", lightNodes.UUID());
  //   point_light["position"] = p1;
  //   point_light["color"] = options.set_light_color ? options.light_color : glm::vec3(0.5);

  //   point_light = cntxt->createNodeFromType("PointLight", "p2", lightNodes.UUID());
  //   point_light["position"] = p2;
  //   point_light["color"] = options.set_light_color ? options.light_color : glm::vec3(0.5);

  //   point_light = cntxt->createNodeFromType("PointLight", "p3", lightNodes.UUID());
  //   point_light["position"] = p3;
  //   point_light["color"] = options.set_light_color ? options.light_color : glm::vec3(0.5);

  //   point_light = cntxt->createNodeFromType("PointLight", "p4", lightNodes.UUID());
  //   point_light["position"] = p4;
  //   point_light["color"] = options.set_light_color ? options.light_color : glm::vec3(0.5);

  //   point_light = cntxt->createNodeFromType("PointLight", "p5", lightNodes.UUID());
  //   point_light["position"] = p5;
  //   point_light["color"] = options.set_light_color ? options.light_color : glm::vec3(0.5);

  //   point_light = cntxt->createNodeFromType("PointLight", "p6", lightNodes.UUID());
  //   point_light["position"] = p6;
  //   point_light["color"] = options.set_light_color ? options.light_color : glm::vec3(0.5);
  // }

  // // TODO: ambient light is currently not supported
  // // gvt::core::DBNodeH ambient_light = cntxt->createNodeFromType("AmbientLight", "p5", lightNodes.UUID());
  // // ambient_light["color"] = options.set_light_color ? options.light_color : glm::vec3(1.0);

  // // set the camera
  // gvt::core::DBNodeH camNode = cntxt->createNodeFromType("Camera", "cam", root.UUID());

  // g_camera_state.reset(options, *meshbbox);

  // camNode["eyePoint"] = g_camera_state.getPos();
  // camNode["focus"] = g_camera_state.getCenter();
  // camNode["upVector"] = g_camera_state.getUp();
  // camNode["fov"] = (float)(45.0 * M_PI / 180.0);
  // camNode["rayMaxDepth"] = static_cast<int>(options.ray_depth);
  // camNode["raySamples"] = static_cast<int>(options.ray_samples);
  // camNode["jitterWindowSize"] = (float)0;

  // film
  gvt::core::DBNodeH filmNode = cntxt->createNodeFromType("Film", "conefilm", root.UUID());
  filmNode["width"] = options.width;
  filmNode["height"] = options.height;

  gvt::core::DBNodeH schedNode = cntxt->createNodeFromType("Schedule", "Enzosched", root.UUID());
  // if (cmd.isSet("domain"))
  //   schedNode["type"] = gvt::render::scheduler::Domain;
  // else
  //   schedNode["type"] = gvt::render::scheduler::Image;
  if (options.tracer == commandline::Options::ASYNC_DOMAIN || options.tracer == commandline::Options::SYNC_DOMAIN) {
    schedNode["type"] = gvt::render::scheduler::Domain;
  } else {
    schedNode["type"] = gvt::render::scheduler::Image;
  }

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

  // // setup gvtCamera from database entries
  // g_camera = new gvt::render::data::scene::gvtPerspectiveCamera;
  // glm::vec3 cameraposition = camNode["eyePoint"].value().tovec3();
  // glm::vec3 focus = camNode["focus"].value().tovec3();
  // float fov = camNode["fov"].value().toFloat();
  // glm::vec3 up = camNode["upVector"].value().tovec3();
  // int rayMaxDepth = camNode["rayMaxDepth"].value().toInteger();
  // int raySamples = camNode["raySamples"].value().toInteger();
  // g_camera->lookAt(cameraposition, focus, up);
  // g_camera->setMaxDepth(rayMaxDepth);
  // g_camera->setSamples(raySamples);
  // g_camera->setFOV(fov);
  // g_camera->setFilmsize(filmNode["width"].value().toInteger(), filmNode["height"].value().toInteger());
} // void CreateObjDatabase(const commandline::Options& options) {

void Kill() {
  g_worker->Quit();
  g_worker->Wait();
}

float pan_speed = 0.005f;
float move_speed = 1.02f;
void KeyboardFunc(unsigned char key, int x, int y) {

  glm::vec3 eye = g_camera->getEyePoint();
  glm::vec3 focal = g_camera->getFocalPoint();
  glm::vec3 up = g_camera->getUpVector();
  float cam_distance = glm::length(focal - eye);
  glm::vec3 look = glm::normalize(focal - eye);
  glm::vec3 tangent = glm::cross(look, up);

  switch (key) {
  case 'q':
    printf("pressed q\n");
    g_quit_key_entered = true;
    break;
  case 'p': {
    std::cout << "================" << std::endl;
    glm::vec3 campos = g_camera_state.getPos();
    glm::vec3 center = g_camera_state.getCenter();
    std::cout << "eye         -> " << campos[0] << " " << campos[1] << " " << campos[2] << std::endl;
    std::cout << "focal point -> " << center[0] << " " << center[1] << " " << center[2] << std::endl;
    // std::cout << "up vector   -> " << up[0] << " " << up[1] << " " << up[2] << std::endl; }
  } break;

  case ' ':
    g_camera_state.reset(*g_options, g_scene_bound);
    eye = g_camera_state.getPos();
    focal = g_camera_state.getCenter();
    up = g_camera_state.getUp();
    break;

  case 'w': {
    g_camera_state.zoom(0.01f);
    g_camera->lookAt(g_camera_state.getPos(), g_camera_state.getCenter(), g_camera_state.getUp());
  } break;
  case 't': {
    g_camera_state.zoom(-0.01f);
    g_camera->lookAt(g_camera_state.getPos(), g_camera_state.getCenter(), g_camera_state.getUp());
    // glm::vec3 new_eye_pos = move_speed * eye;
    // float r = glm::length(new_eye_pos - g_scene_bound.centroid());
    // float sign = (key == 'w') ? 1.f : -1.f;
    // if (r > glm::length(g_scene_bound.extent()) * 0.001) {
    //   eye += sign * new_eye_pos;
    // }
    // // focal += sign * move_speed * look;
    // break;
  } break;
  case '=': {
    g_camera_state.increaseSensitivity();
    printf("camera sensitivity: %f\n", g_camera_state.getSensitivity());
  } break;
  case '-': {
    g_camera_state.decreaseSensitivity();
    printf("camera sensitivity: %f\n", g_camera_state.getSensitivity());
  } break;

  case 'a':
    eye += -move_speed * tangent;
    focal += -move_speed * tangent;
    break;
  case 's':
    eye += -move_speed * look;
    focal += -move_speed * look;
    break;
  case 'd':
    eye += move_speed * tangent;
    focal += move_speed * tangent;
    break;
  case 'j': // move up
    eye += -move_speed * up;
    focal += -move_speed * up;
    break;
  case 'k': // move down
    eye += move_speed * up;
    focal += move_speed * up;
    break;
  case 'n': // turn left
    look = glm::rotate(look, pan_speed, up);
    focal = eye + cam_distance * look;
    break;
  case 'm': // turn right
    look = glm::rotate(look, -pan_speed, up);
    focal = eye + cam_distance * look;
    break;
  case 'u': // look down
    look = glm::rotate(look, pan_speed, tangent);
    focal = eye + cam_distance * look;
    break;
  case 'i': // look up
    look = glm::rotate(look, -pan_speed, tangent);
    focal = eye + cam_distance * look;
    break;

  default:
    break;
  }
  g_camera->lookAt(eye, focal, up);
}

bool g_mouse_left = false;
bool g_mouse_right = false;
int g_mouse_x = 0;
int g_mouse_y = 0;
float orbit_speed = 0.01f;

static void MouseFunc(int button, int state, int x, int y) {
  if (state == GLUT_DOWN) {
    g_mouse_x = x;
    g_mouse_y = y;
    if (button == GLUT_LEFT_BUTTON) {
      g_mouse_left = true;
    } else if (button == GLUT_RIGHT_BUTTON) {
      g_mouse_right = true;
    } else if (button == 3 || button == 4) {
      g_camera_state.zoom(button == 3 ? 0.01f : -0.01f);
      g_camera->lookAt(g_camera_state.getPos(), g_camera_state.getCenter(), g_camera_state.getUp());
    }
  } else if (state == GLUT_UP) {
    if (button == GLUT_LEFT_BUTTON) {
      g_mouse_left = false;
    }
  }
}

static void MotionFunc(int x, int y) {
  int dx = x - g_mouse_x;
  int dy = y - g_mouse_y;

  if (g_mouse_left) {
    g_camera_state.rotate(dx, dy, g_options->width, g_options->height);
  } else if (g_mouse_right) {
    g_camera_state.pan(dx, dy, g_camera->getCameraToWorld());
  }

  g_camera->lookAt(g_camera_state.getPos(), g_camera_state.getCenter(), g_camera_state.getUp());

  g_mouse_x = x;
  g_mouse_y = y;
}

void DisplayFunc(void) {
  static bool quit = false;
  static double st = 0.0;

  if (g_num_frames == g_num_warmup_frames) {
    st = MPI_Wtime();
  }

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  if (g_num_frames == (g_num_warmup_frames + g_num_active_frames) || quit) {
    double elapsed = MPI_Wtime() - st;
    double fps = static_cast<double>(g_num_frames - g_num_warmup_frames) / elapsed;
    std::cout << "elapsed time: " << elapsed << " seconds per " << g_num_frames << " frames (" << fps << " fps) "
              << std::endl;
    glutDestroyWindow(g_window);
    Kill();
    MPI_Finalize();
    exit(0);
  }

  g_camera->AllocateCameraRays();
  g_camera->generateRays();
  g_image->clear();
  g_worker->Render();

  ++g_num_frames;

  if (g_quit_key_entered) quit = true;

  // PrintHelpAndSettings();
  // glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  // glRasterPos2i(0, 0);
  glDrawPixels(g_width, g_height, GL_RGB, GL_UNSIGNED_BYTE, g_image->GetBuffer());
  glutSwapBuffers();
  glutPostRedisplay();
}

void InitGlut(int width, int height) {
  // resizeDisplay(width, height);
  int argc = 0;
  char **argv = NULL;
  glutInit(&argc, argv);
  glutInitWindowSize((GLsizei)width, (GLsizei)height);
  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE);
  glutInitWindowPosition(5, 5);
  g_window = glutCreateWindow("MpiApp");
  glutDisplayFunc(DisplayFunc);
  // glutIdleFunc(idleFunc);
  glutKeyboardFunc(KeyboardFunc);
  // glutSpecialFunc(SpecialFunc);
  glutMouseFunc(MouseFunc);
  glutMotionFunc(MotionFunc);
  // glutReshapeFunc(reshapeFunc);
  glutMainLoop();
}

void CreateTracer(const commandline::Options &options, const gvt::render::unit::MpiInfo &mpi) {
  switch (options.tracer) {
  case commandline::Options::ASYNC_DOMAIN: {
    if (mpi.rank == 0) std::cout << "start ASYNC_DOMAIN" << std::endl;
    g_image = new Image(g_camera->getFilmSizeWidth(), g_camera->getFilmSizeHeight(), GetTestName(mpi, options));
    g_worker = new Worker(mpi, options, g_camera, g_image);
  } break;

  // case commandline::Options::SYNC_DOMAIN: {
  //   if (mpi.rank == 0) std::cout << "start SYNC_DOMAIN" << std::endl;
  //   g_image = new Image(g_camera->getFilmSizeWidth(), g_camera->getFilmSizeHeight(), GetTestName(mpi, options));
  //   g_tracer = new gvt::render::algorithm::Tracer<DomainScheduler>(g_camera->rays, *g_image);
  // } break;

  default: {
    std::cout << "rank " << mpi.rank << " error found unsupported tracer type " << options.tracer << std::endl;
    exit(1);
  } break;
  }
}

void RenderInteractive(const commandline::Options &options, const gvt::render::unit::MpiInfo &mpi) {
  CreateTracer(options, mpi);
  if (mpi.rank == 0) {
    InitGlut(options.width, options.height);
  } else {
    // bool quit_pressed = false;
    for (int i = 0; i < g_num_warmup_frames + g_num_active_frames; ++i) {
      g_camera->AllocateCameraRays();
      g_camera->generateRays();
      g_image->clear();
      g_worker->Render();
      // g_worker->IsQuit(&quit_pressed);
      // if (quit_pressed) break;
    }
    g_worker->Wait();
  }
}

void RenderFilm(const commandline::Options &options, gvt::render::unit::MpiInfo &mpi) {
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

    g_image = new Image(g_camera->getFilmSizeWidth(), g_camera->getFilmSizeHeight(), GetTestName(mpi, options));

    Worker worker(mpi, options, g_camera, g_image);

    DomainTracer *domain_tracer = static_cast<DomainTracer *>(worker.GetTracer());
    Profiler &profiler = *domain_tracer->getProfiler();

    gvt::render::RenderContext *cntxt = gvt::render::RenderContext::instance();
    gvt::core::DBNodeH root = cntxt->getRootNode();
#ifdef PRINT_NUM_DOMAINS
    std::cout << "[INFO] rank " << mpi.rank << " num domains: " << root["Instances"].getChildren().size() << "\n";
#endif
    profiler.SetNumDomains(root["Instances"].getChildren().size());

    // warm up
    for (int i = 0; i < options.warmup_frames; i++) {
      g_camera->AllocateCameraRays();
      g_camera->generateRays();
      g_image->clear();
      worker.Render();
#ifdef PRINT_FRAME_NUMBER
      std::cout << "rank " << mpi.rank << " warm up frame " << i << " done\n\n";
#endif
    }

    profiler.Reset();
    profiler.Start(Profiler::TOTAL_TIME);

    for (int i = 0; i < options.active_frames; i++) {
      profiler.Start(Profiler::CAMERA_RAY);
      g_camera->AllocateCameraRays();
      g_camera->generateRays();
      g_image->clear();
      profiler.Stop(Profiler::CAMERA_RAY);

      worker.Render();
#ifdef PRINT_FRAME_NUMBER
      std::cout << "rank " << mpi.rank << " active frame " << i << " done\n\n";
#endif
    }

    profiler.Stop(Profiler::TOTAL_TIME);

    if (mpi.rank == 0) {
      g_image->Write();
      worker.Quit();
    }
    worker.Wait();

    profiler.WriteToFile(GetTestName(mpi, options) + ".txt", mpi.rank);

  } break;

  case commandline::Options::SYNC_DOMAIN: {
    if (mpi.rank == 0) std::cout << "start SYNC_DOMAIN" << std::endl;

    g_image = new Image(g_camera->getFilmSizeWidth(), g_camera->getFilmSizeHeight(), GetTestName(mpi, options));

    g_camera->AllocateCameraRays();
    g_camera->generateRays();

    gvt::render::algorithm::Tracer<DomainScheduler> tracer(g_camera->rays, *g_image);
    Profiler &profiler = *tracer.getProfiler();

    for (int z = 0; z < options.warmup_frames; z++) {
      g_camera->AllocateCameraRays();
      g_camera->generateRays();
      g_image->clear();
      tracer();
    }

    profiler.Reset();
    profiler.Start(Profiler::TOTAL_TIME);
    for (int z = 0; z < options.active_frames; z++) {
      profiler.Start(Profiler::CAMERA_RAY);
      g_camera->AllocateCameraRays();
      g_camera->generateRays();
      g_image->clear();
      profiler.Stop(Profiler::CAMERA_RAY);

      tracer();
    }
    profiler.Stop(Profiler::TOTAL_TIME);

    g_image->Write();

    profiler.WriteToFile(GetTestName(mpi, options) + ".txt", mpi.rank);

  } break;

  case commandline::Options::SYNC_IMAGE: {
    if (mpi.rank == 0) std::cout << "start SYNC_IMAGE" << std::endl;

    g_image = new Image(g_camera->getFilmSizeWidth(), g_camera->getFilmSizeHeight(), GetTestName(mpi, options));

    g_camera->AllocateCameraRays();
    g_camera->generateRays();

    gvt::render::algorithm::Tracer<ImageScheduler> tracer(g_camera->rays, *g_image);
    Profiler &profiler = *tracer.getProfiler();

    for (int z = 0; z < options.warmup_frames; z++) {
      g_camera->AllocateCameraRays();
      g_camera->generateRays();
      g_image->clear();
      tracer();
    }

    profiler.Reset();
    profiler.Start(Profiler::TOTAL_TIME);
    for (int z = 0; z < options.active_frames; z++) {
      profiler.Start(Profiler::CAMERA_RAY);
      g_camera->AllocateCameraRays();
      g_camera->generateRays();
      g_image->clear();
      profiler.Stop(Profiler::CAMERA_RAY);

      tracer();
    }
    profiler.Stop(Profiler::TOTAL_TIME);

    g_image->Write();

    profiler.WriteToFile(GetTestName(mpi, options) + ".txt", mpi.rank);

  } break;

  default: {
    std::cout << "rank " << mpi.rank << " error found unsupported tracer type " << options.tracer << std::endl;
    exit(1);
  } break;
  }
  // clean up
  if (g_image) delete g_image;
  if (g_camera) delete g_camera;
}

} // namespace mpi
} // namespace render
} // namespace apps

using namespace gvt::render::unit;
using namespace apps::render::mpi;

int main(int argc, char **argv) {
  int pvd;
  MPI_Init_thread(&argc, &argv, MPI_THREAD_MULTIPLE, &pvd);
  if ((pvd != MPI_THREAD_MULTIPLE)) {
    std::cerr << "error: mpi_thread_multiple not available\n";
    exit(1);
  }

  MpiInfo mpi;
  MPI_Comm_rank(MPI_COMM_WORLD, &mpi.rank);
  MPI_Comm_size(MPI_COMM_WORLD, &mpi.size);

  if (mpi.rank == 0) std::cout << "mpi size: " << mpi.size << std::endl;

  commandline::Options options;
  commandline::Parse(argc, argv, &options);

  g_width = options.width;
  g_height = options.height;
  g_tracer_mode = options.tracer;
  g_mpiRank = mpi.rank;
  g_mpiSize = mpi.size;
  g_num_warmup_frames = options.warmup_frames;
  g_num_active_frames = options.active_frames;
  g_options = &options;

  // initialize tbb
  // tbb::task_scheduler_init init;
  // tbb::task_scheduler_init init(tbb::task_scheduler_init::default_num_threads());
  tbb::task_scheduler_init init(options.numTbbThreads);

  // create database
  if (options.tracer != commandline::Options::PING_TEST) {
    timer t_database(false, "database timer:");
    std::cout << "rank " << mpi.rank << " creating database." << std::endl;
    t_database.start();
    if (options.obj) {
      CreateObjDatabase(mpi, options);
    } else {
      CreatePlyDatabase(mpi, options);
    }
    t_database.stop();
    std::cout << "rank " << mpi.rank << " done creating database." << std::endl;
  }

  if (options.interactive) {
    RenderInteractive(options, mpi);
  } else {
    RenderFilm(options, mpi);
  }
  MPI_Finalize();
}

