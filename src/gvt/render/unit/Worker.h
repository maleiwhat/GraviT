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
// Worker.h
//

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
class Voter;
class Work;

class Worker {
public:
  // Worker(int* argc, char*** argv, const commandline::Options& options,
  Worker(const MpiInfo &mpi, const commandline::Options &options, gvtPerspectiveCamera *camera, Image *image);

  ~Worker();

  void Render();

  // sync
  void Quit();
  void Wait();

  // get mpi info
  MpiInfo GetMpiInfo() const { return mpi; }
  int GetRank() const { return mpi.rank; }
  int GetMpiSize() const { return mpi.size; }

  // get units
  Communicator *GetCommunicator() const { return comm; }
  RayTracer *GetTracer() { return tracer; }
  Voter *GetVoter() { return voter; }

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
  Communicator *comm;

  // scene
  gvtPerspectiveCamera *camera;
  Image *image;

  // processors
  RayTracer *tracer;
  Voter *voter;

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

