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
  // Worker(int* argc, char*** argv, const commandline::Options& options,
  Worker(const MpiInfo& mpi, const commandline::Options& options,
         gvtPerspectiveCamera* camera, Image* image);

  ~Worker();

  // void InitTracer(const commandline::Options& options,
  //                 gvtPerspectiveCamera* camera, Image* image);
  void Render();

  // sync
  void Quit();
  void Wait();

  // get mpi info
  MpiInfo GetMpiInfo() const { return mpi; }
  int GetRank() const { return mpi.rank; }
  int GetMpiSize() const { return mpi.size; }

  // get units
  Communicator* GetCommunicator() const { return comm; }
  RayTracer* GetTracer() { return tracer; }
  TpcVoter* GetVoter() { return voter; }


 private:
  friend class Command;
  void QuitCommunicator();

  friend class Communicator;

  void SignalMpiReady();
  void SignalTracerReady();

  void WaitMpiReady();
  void WaitTracerReady();

 private:
  // communication
  MpiInfo mpi;
  Communicator* comm;

  // scene
  gvtPerspectiveCamera* camera;
  Image* image;

  // processors 
  RayTracer* tracer;
  TpcVoter* voter;

  bool mpiReady;
  pthread_mutex_t mpiReady_mutex;
  pthread_cond_t mpiReady_cond;

  bool tracerReady;
  pthread_mutex_t tracerReady_mutex;
  pthread_cond_t tracerReady_cond;

  bool quit;
  pthread_mutex_t quit_mutex;
  pthread_cond_t quit_cond;
};

} // using namespace unit
} // using namespace render
} // using namespace gvt

#endif

