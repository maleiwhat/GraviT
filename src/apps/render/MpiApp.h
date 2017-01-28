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

  int tracer;
  int adapter;
  int width;
  int height;
  bool obj;
  int instanceCountX;
  int instanceCountY;
  int instanceCountZ;
  int numFrames;
  int numTbbThreads;
  std::string infile;
  std::string model_name;
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
  float fov;
  int ray_depth;
  int ray_samples;
  int warmup_frames;
  int active_frames;
  bool interactive;
  std::vector<PointLightInfo> point_lights;
};

}  // namespace commandline
}  // namespace mpi
}  // namespace render
}  // namespace apps

#endif
