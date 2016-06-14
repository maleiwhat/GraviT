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

Worker::Worker(const MpiInfo& mpi, const commandline::Options& options,
               gvtPerspectiveCamera* camera, Image* image)
    : mpi(mpi), camera(camera), image(image), quit(false) {
  // init mutex
  pthread_mutex_init(&quit_mutex, NULL);
  pthread_cond_init(&quit_cond, NULL);

  // create communicator
  comm = new Communicator(mpi, this);

  Command::Register(comm);

  // create tracer and register required works
  tracer = NULL;
  switch (options.tracer) {
    case commandline::Options::PING_TEST: {
      tracer = new PingTracer(mpi, this, comm);
      // register works
      PingTest::Register(comm);

    } break;
    case commandline::Options::ASYNC_DOMAIN: {
      tracer = new DomainTracer(mpi, this, comm, camera->rays, *image);

      // register works
      RemoteRays::Register(comm);
      Vote::Register(comm);

    } break;
    default: {
      std::cout << "rank " << mpi.rank
                << " error found unsupported tracer type " << options.tracer
                << std::endl;
      exit(1);
    } break;
  }

  // voter owned by tracer
  voter = tracer->GetVoter();
}

Worker::~Worker() {
  delete comm;
  if (tracer) delete tracer;
}

void Worker::Render() { tracer->Render(); }

void Worker::Quit() {
  if (mpi.rank == 0) {
    Work* work = new Command(Command::QUIT);
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

}  // using namespace unit
}  // using namespace render
}  // using namespace gvt

