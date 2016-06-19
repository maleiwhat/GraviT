#ifndef APPS_RENDER_MPI_APP_H
#define APPS_RENDER_MPI_APP_H

#include <string>
#include <glm/glm.hpp>

namespace apps {
namespace render {
namespace mpi {
namespace commandline {

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
  glm::vec3 light_color;
  glm::vec3 eye;
  glm::vec3 look;
  glm::vec3 up;
  float fov;
  int ray_max_depth;
  int ray_samples;
};

}  // namespace commandline
}  // namespace mpi
}  // namespace render
}  // namespace apps

#endif
