#ifndef GVT_RENDER_UNIT_WORKER_H
#define GVT_RENDER_UNIT_WORKER_H

#include <pthread.h>

#include "gvt/render/unit/Types.h"
#include "gvt/render/unit/RayTracer.h"

namespace apps {
namespace render {
namespace mpi {
namespace commandline {

struct Options;
}
}
}
}

namespace gvt {
namespace render {
namespace data {
namespace scene {

class gvtPerspectiveCamera;
class Image;

}
}
}
}

namespace gvt {
namespace render {
namespace unit {

using namespace apps::render::mpi;
using namespace gvt::render::data::scene;

class Communicator;
// class RayTracer;
class TpcVoter;

class Worker {
 public:
  Worker(const MpiInfo& mpi, const commandline::Options& options,
         gvtPerspectiveCamera* camera, Image* image);

  ~Worker();

  void Render();

  void Quit();
  void Wait();

  // get units
  Communicator* GetCommunicator() const { return comm; }
  RayTracer* GetTracer() { return tracer; }
  TpcVoter* GetVoter() { return voter; }

  // get mpi info
  int GetRank() const { return mpi.rank; }
  int GetMpiSize() const { return mpi.size; }

 private:
  friend class Command;
  void QuitCommunicator();

 private:
  // communication
  const MpiInfo mpi;
  Communicator* comm;

  // scene
  gvtPerspectiveCamera* camera;
  Image* image;

  // processors 
  RayTracer* tracer;
  TpcVoter* voter;

  bool quit;
  pthread_mutex_t quit_mutex;
  pthread_cond_t quit_cond;
};

} // using namespace unit
} // using namespace render
} // using namespace gvt

#endif

