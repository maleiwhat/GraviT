#ifndef APPS_RENDER_MPI_APP_H
#define APPS_RENDER_MPI_APP_H

#include <string>
#include <glm/glm.hpp>

namespace apps {
namespace render {
namespace mpi {
namespace commandline {

struct PointLightInfo {
  glm::vec3 position;
  glm::vec3 color;
};

struct Options {
  enum TracerType {
    ASYNC_DOMAIN = 0,
    ASYNC_IMAGE,
    SYNC_DOMAIN,
    SYNC_IMAGE,
    PING_TEST,
    NUM_TRACERS
  };

  enum AdapterType { EMBREE, MANTA, OPTIX };

  int tracer = ASYNC_DOMAIN;
  int adapter = EMBREE;
  int width = 400;
  int height = 400;
  bool obj = false;
  int instanceCountX = 1;
  int instanceCountY = 1;
  int instanceCountZ = 1;
  int numFrames = 1;
  int numTbbThreads = -1;
  std::string infile;
  std::string model_name = std::string("unknownmodel");
  glm::vec3 light_position;
  bool set_light_position = false;
  glm::vec3 light_color;
  bool set_light_color = false;
  glm::vec3 eye;
  bool set_eye = false;
  glm::vec3 look;
  bool set_look = false;
  glm::vec3 up;
  bool set_up = false;
  float fov = 45;
  int ray_depth = 1;
  int ray_samples = 1;
  int warmup_frames = 1;
  int active_frames = 1;
  bool interactive = false;
  std::vector<PointLightInfo> point_lights;
};

}  // namespace commandline
}  // namespace mpi
}  // namespace render
}  // namespace apps

#endif
