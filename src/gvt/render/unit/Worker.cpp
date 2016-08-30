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
// Worker.cpp
//

#include "gvt/render/unit/Worker.h"

#include "apps/render/MpiApp.h"

#include "gvt/render/unit/Communicator.h"

#include "gvt/render/unit/DomainTracer.h"
#include "gvt/render/unit/TestTracer.h"

#include "gvt/render/unit/CommonWorks.h"
#include "gvt/render/unit/DomainWorks.h"

#include "gvt/render/data/scene/gvtCamera.h"

namespace gvt {
namespace render {
namespace unit {

using namespace apps::render::mpi;
using namespace gvt::render::data::scene;

Worker::Worker(const MpiInfo &mpi, const commandline::Options &options, gvtPerspectiveCamera *camera, Image *image)
    : mpi(mpi), camera(NULL), image(NULL), quit(false), mpiReady(false), tracerReady(false) {
  // init mutex
  pthread_mutex_init(&quit_mutex, NULL);
  pthread_cond_init(&quit_cond, NULL);

  pthread_mutex_init(&mpiReady_mutex, NULL);
  pthread_cond_init(&mpiReady_cond, NULL);

  pthread_mutex_init(&tracerReady_mutex, NULL);
  pthread_cond_init(&tracerReady_cond, NULL);

  // create communicator
  comm = new Communicator(mpi, this);

  // wait until mpi gets initialized
  // WaitMpiReady();

  // mpi info is available from the communicator
  // mpi = comm->GetMpiInfo();

  // all applications require this for quitting the worker
  Command::Register(comm);

  // create tracer and register required works
  tracer = NULL;
  switch (options.tracer) {
  case commandline::Options::ASYNC_DOMAIN: {
    tracer = new DomainTracer(mpi, this, comm, camera->rays, *image);
    RemoteRays::Register(comm);
    Vote::Register(comm);
    Composite::Register(comm);
  } break;

  case commandline::Options::PING_TEST: {
    tracer = new PingTracer(mpi, this, comm);
    PingTest::Register(comm);
  } break;

  default: {
    std::cout << "rank " << mpi.rank << " error found unsupported tracer type " << options.tracer << std::endl;
    exit(1);
  } break;
  }

  // voter owned by tracer
  voter = tracer->getVoter();

  // finally let the communicator serve incoming message
  SignalTracerReady();
}

Worker::~Worker() {
  delete comm;
  if (tracer) delete tracer;
}

void Worker::Render() { tracer->trace(); }

void Worker::Quit() {
  if (mpi.rank == 0) {
    Work *work = new Command(Command::QUIT);
    work->SendAll(comm);
  }
}

void Worker::QuitCommunicator() {
  comm->Quit();
  pthread_mutex_lock(&quit_mutex);
  quit = true;
  pthread_cond_signal(&quit_cond);
  pthread_mutex_unlock(&quit_mutex);
}

void Worker::Wait() {
  pthread_mutex_lock(&quit_mutex);
  while (!quit) {
    pthread_cond_wait(&quit_cond, &quit_mutex);
  }
  quit = false;
  pthread_mutex_unlock(&quit_mutex);
}

void Worker::SignalMpiReady() {
  pthread_mutex_lock(&mpiReady_mutex);
  mpiReady = true;
  pthread_cond_signal(&mpiReady_cond);
  pthread_mutex_unlock(&mpiReady_mutex);
}

void Worker::SignalTracerReady() {
  pthread_mutex_lock(&tracerReady_mutex);
  tracerReady = true;
  pthread_cond_signal(&tracerReady_cond);
  pthread_mutex_unlock(&tracerReady_mutex);
}

void Worker::WaitMpiReady() {
  pthread_mutex_lock(&mpiReady_mutex);
  while (!mpiReady) {
    pthread_cond_wait(&mpiReady_cond, &mpiReady_mutex);
  }
  mpiReady = false;
  pthread_mutex_unlock(&mpiReady_mutex);
}

void Worker::WaitTracerReady() {
  pthread_mutex_lock(&tracerReady_mutex);
  while (!tracerReady) {
    pthread_cond_wait(&tracerReady_cond, &tracerReady_mutex);
  }
  tracerReady = false;
  pthread_mutex_unlock(&tracerReady_mutex);
}

} // using namespace unit
} // using namespace render
} // using namespace gvt

