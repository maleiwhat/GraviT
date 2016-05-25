#pragma once

#include <pthread.h>
#include <vector>
#include <unistd.h>
#include <sys/types.h>
#include "vector"
#include "sstream"
#include "RayQManager.h"
#include "MessageManager.h"
#include "MessageQ.h"
#include "Work.h"

namespace gvt {
namespace core {
namespace mpi {

class Application {

public:
  Application(int *argc, char ***argv);
  ~Application();

  static Application *GetTheApplication() { return Application::theApplication; }
  MessageManager *GetTheMessageManager() { return theMessageManager; }
  RayQManager *GetTheRayQManager() { return theRayQManager; }

  void QuitApplication();

  int *GetPArgC() { return argcp; }
  char ***GetPArgV() { return argvp; }

  int GetSize() { return theMessageManager->GetSize(); }
  int GetRank() { return theMessageManager->GetRank(); }

  void Start();
  void Kill();
  void Wait();
  bool Running() { return !application_done; }

  void log(std::stringstream &s);

  Work *Deserialize(Message *msg);

  int RegisterWork(Work *(*f)(SharedP)) {
    int n = deserializers.size();
    deserializers.push_back(f);
    return n;
  }

  bool IsDoneSet() { return application_done; }
  pid_t get_pid() { return pid; }

private:
  static Application *theApplication;

  RayQManager *theRayQManager;
  MessageManager *theMessageManager;

  vector<Work *(*)(SharedP)> deserializers;

  bool application_done;

  int *argcp;
  char ***argvp;

  pid_t pid;

  pthread_mutex_t lock;
  pthread_cond_t cond;

private:
  class Quit : public Work {
    WORK_CLASS(Quit, true)

  public:
    bool Action(MPI_Comm coll_comm);
  };
};
}
}
}
