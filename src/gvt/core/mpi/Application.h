#ifndef GVT_CORE_MPI_APPLICATION_H
#define GVT_CORE_MPI_APPLICATION_H

#include "pthread.h"
#include "vector"
#include <gvt/core/mpi/Message.h>
#include <gvt/core/mpi/MessageQ.h>
#include <gvt/core/mpi/Work.h>

#define DEBUG_QUIT

namespace gvt {
namespace core {
namespace mpi {

class Application {
public:
  Application(int *argc, char ***argv);
  ~Application();

  static Application *GetApplication();

  void QuitApplication();

  int *GetPArgC() { return argcp; }
  char ***GetPArgV() { return argvp; }

  int GetSize() { return GetMessageManager()->GetSize(); }
  int GetRank() { return GetMessageManager()->GetRank(); }

  void Start();
  void Kill();
  void Wait();
  bool Running() { return !application_done; }

  MessageQ *GetIncomingMessageQueue() { return theIncomingQueue; }
  MessageQ *GetOutgoingMessageQueue() { return theOutgoingQueue; }

  Work *Deserialize(Message *message) {
    return deserializers[message->GetType()](message->GetSize(), message->GetBytes());
  }

  int RegisterWork(Work *(*f)(size_t size, unsigned char *serialized)) {
    int n = deserializers.size();
    deserializers.push_back(f);
    return n;
  }

  MessageManager *GetMessageManager() { return &theMessageManager; }

  bool IsDoneSet() { return application_done; }

private:
  vector<Work *(*)(size_t, unsigned char *)> deserializers;

  static void *workThread(void *);
  pthread_t work_thread_tid;

  MessageManager theMessageManager;

  MessageQ *theIncomingQueue;
  MessageQ *theOutgoingQueue;

  bool application_done;

  int *argcp;
  char ***argvp;

  pthread_mutex_t lock;
  pthread_cond_t cond;
};

class Quit : public Work {
  WORK_CLASS_HEADER(Quit)

public:
  static Work *Deserialize(size_t size, unsigned char *serialized) {
#ifdef DEBUG_QUIT
    printf("Rank %d: Quit::Deserialize.\n", Application::GetApplication()->GetRank());
#endif
    Quit *bcast = new Quit;
    return (Work *)bcast;
  }

  void Serialize(size_t &size, unsigned char *&serialized) {
#ifdef DEBUG_QUIT
    printf("Rank %d: Quit::Serialize.\n", Application::GetApplication()->GetRank());
#endif
    size = 0;
    serialized = NULL;
  }

  bool Action() {
#ifdef DEBUG_QUIT
    printf("Rank %d: Quit::Action. quitting the app.\n", Application::GetApplication()->GetRank());
#endif
    return true;
  }
};

} // ns mpi
} // ns core
} // ns gvt

#endif /* GVT_CORE_MPI_APPLICATION_H */
