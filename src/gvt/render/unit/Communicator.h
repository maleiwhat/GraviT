#ifndef GVT_RENDER_UNIT_COMMUNICATOR_H
#define GVT_RENDER_UNIT_COMMUNICATOR_H

#include "gvt/render/unit/Types.h"
#include "gvt/render/unit/Contents.h"
#include "gvt/render/unit/RayTracer.h"

#include <pthread.h>
#include <queue>
#include <memory>
#include <mpi.h>

namespace gvt {
namespace render {
namespace unit {

class Work;
class Worker;
class TpcVoter;

class Communicator {
 public:
  Communicator(int* argc, char*** argv, const MpiInfo& mpi, Worker* worker);
  ~Communicator() { MPI_Finalize(); }

  // register deserializers
  int RegisterWork(Work* (*Deserialize)());

  // helper functions
  // void SetDone() { allWorkDone = true; }
  void Quit();

  // mpi
  void Send(Work* work);

  MpiInfo GetMpiInfo() const { return mpi; }
  int GetRank() { return mpi.rank; }
  int GetMpiSize() { return mpi.size; }
  // MPI_Comm GetMpiComm() { return mpi.comm; }

 private:
  enum ThreadId { MESSAGE_THREAD = 0, WORK_THREAD, NUM_PTHREADS };

  Worker* worker;

  int* argcp;
  char*** argvp;
  // intializers
  // void Initialize();
  // void InitMpi();
  void InitThreads();
  void InitMutexes();

  // p-threads start routines
  static void* StartMessageThread(void* This);
  static void* StartWorkThread(void* This);

  // p-threads
  void MessageThread();
  void WorkThread();

  // mpi rank, size
  MpiInfo mpi;

  // mpi send/recv
  void RecvWork(const MPI_Status& status, Work* work);
  void SendWork(Work* work);
  void SendWorkAllOther(Work* work);

  void IsendWork(Work* work);

  // work queues for communication
  std::queue<Work*> sendQ;
  std::queue<Work*> recvQ;
  pthread_mutex_t sendQ_mutex;
  pthread_mutex_t recvQ_mutex;

  // flag to indicate there is no more work
  // don't need a lock since worker will set it via ThreadJoin()
  bool allWorkDone;

  // pthreads
  std::vector<pthread_t> threads;

  // deserializers used upon receiving a new MPI message
  std::vector<Work* (*)()> deserializers;
};

}  // namespace unit
}  // namespace render
}  // namespace gvt

#endif
