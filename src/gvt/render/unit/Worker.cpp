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

Worker::Worker(int *argc, char ***argv, const commandline::Options &options,
               gvtPerspectiveCamera *camera, Image *image)
    : camera(NULL), image(NULL), quit(false), mpiReady(false) {
  // init mutex
  pthread_mutex_init(&quit_mutex, NULL);
  pthread_cond_init(&quit_cond, NULL);

  // create communicator
  comm = new Communicator(argc, argv, this);

  // wait until mpi gets initialized
  pthread_mutex_lock(&mpiReady_mutex);
  while (!mpiReady) {
    pthread_cond_wait(&mpiReady_cond, &mpiReady_mutex);
  }
  mpiReady = false;
  pthread_mutex_unlock(&mpiReady_mutex);

  // mpi info is available from the communicator
  mpi = comm->GetMpiInfo();
#if 0
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
      std::cout << "rank " << mpi.rank
                << " error found unsupported tracer type " << options.tracer
                << std::endl;
      exit(1);
    } break;
  }

  // voter owned by tracer
  voter = tracer->GetVoter();
#endif
}

Worker::~Worker() {
  delete comm;
  if (tracer) delete tracer;
}

void Worker::InitTracer(const commandline::Options &options,
                        gvtPerspectiveCamera *camera, Image *image) {
  this->camera = camera;
  this->image = image;

  // all applications require this for quitting the worker
  Command::Register(comm);

  // create tracer and register required works
  tracer = NULL;
  switch (options.tracer) {
    case commandline::Options::ASYNC_DOMAIN: {
      std::cout << "rank " << mpi.rank << " creating domain tracer"
                << std::endl;
      tracer = new DomainTracer(mpi, this, comm, this->camera->rays, *this->image);
      RemoteRays::Register(comm);
      Vote::Register(comm);
      Composite::Register(comm);
    } break;

    case commandline::Options::PING_TEST: {
      tracer = new PingTracer(mpi, this, comm);
      PingTest::Register(comm);
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

void Worker::Render() { tracer->Trace(); }

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

void Worker::SignalMpiReady() {
  pthread_mutex_lock(&mpiReady_mutex);
  mpiReady = true;
  pthread_cond_signal(&mpiReady_cond);
  pthread_mutex_unlock(&mpiReady_mutex);
}

}  // using namespace unit
}  // using namespace render
}  // using namespace gvt

