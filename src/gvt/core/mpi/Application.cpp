#include "Application.h"
#include "Rays.h"
#include <fstream>
#include <iostream>
#include <mpi.h>
#include <sstream>
#include <unistd.h>

using namespace gvt::core::mpi;

WORK_CLASS_TYPE(Application::Quit)

Application *Application::theApplication;

Application::Application(int *a, char ***b) {
  Application::theApplication = this;
  pid = getpid();

  application_done = false;

  argcp = a;
  argvp = b;

  pthread_mutex_init(&lock, NULL);
  pthread_cond_init(&cond, NULL);

  pthread_mutex_lock(&lock);

  Quit::Register();
  Rays::Register();

  pthread_mutex_unlock(&lock);

  theMessageManager = new MessageManager();
  theRayQManager = new RayQManager();
}

Application::~Application() {
  pthread_mutex_unlock(&lock);

  delete theRayQManager;
  delete theMessageManager;
}

static pthread_mutex_t log_lock = PTHREAD_MUTEX_INITIALIZER;

void Application::log(std::stringstream &s) {
  pthread_mutex_lock(&log_lock);
  std::fstream fs;
  std::stringstream fname;
  int rank = theMessageManager->GetRank();
  fname << "log_" << rank;
  fs.open(fname.str().c_str(), std::fstream::in | std::fstream::out | std::fstream::app);
  fs << rank << ": " << s.str();
  fs.close();
  pthread_mutex_unlock(&log_lock);
}

void Application::QuitApplication() {
  Quit *quit = new Quit(0);
  theMessageManager->BroadcastWork(quit, true);
}

void Application::Start() { theMessageManager->Start(); }

void Application::Kill() {
  application_done = true;
  pthread_cond_broadcast(&cond);
  pthread_mutex_unlock(&lock);
}

bool Application::Quit::Action(MPI_Comm coll_comm) {
  Application::GetTheApplication()->GetTheRayQManager()->Kill();
  // MPI_Barrier(coll_comm);
  return true;
}

Work *Application::Deserialize(Message *msg) { return deserializers[msg->GetType()](msg->ShareContent()); }

void Application::Wait() { theMessageManager->Wait(); }

