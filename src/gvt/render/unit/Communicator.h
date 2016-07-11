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
  Communicator(const MpiInfo& mpi, Worker* worker);

  /**
   * Register a function pointer to message deserializer defined in the work class.
   */
  int RegisterWork(Work* (*Deserialize)());

  // helper functions
  // void SetDone() { allWorkDone = true; }
  
  /**
   * Kill all the pthreads and check if all the message queues are empty or not.
   */
  void Quit();

  // mpi
 
  /**
   * Send a work to node specified by dest.
   */
  void Send(int dest, Work* work);

  /**
   * Send a work to all nodes including the node that calls this function.
   */
  void SendAll(Work* work);

  /**
   * Send a work to all the rest nodes excluding the node that class this function.
   */
  void SendAllOther(Work* work);

  /**
   * Get MPI rank and world size.
   */
  MpiInfo GetMpiInfo() const { return mpi; }

  /**
   * Get MPI rank.
   */
  int GetRank() { return mpi.rank; }

  /**
   * Get MPI world size.
   */
  int GetMpiSize() { return mpi.size; }
  // MPI_Comm GetMpiComm() { return mpi.comm; }

 private:
  /**
   * Unique IDs associated with created p-threads for MPI communication.
   */
  enum ThreadId { MESSAGE_THREAD = 0, WORK_THREAD, NUM_PTHREADS };

  /**
   * A pointer to the worker object. This is used to call the Action function in the work class.
   */
  Worker* worker;

  int* argcp; ///< unused
  char*** argvp; ///< unused

  // intializers
  // void Initialize();
  // void InitMpi();

  /**
   * Creates p-threads for MPI commnication.
   */
  void InitThreads();

  /**
   * Initializes required mutexes.
   */
  void InitMutexes();

  /**
   * P-thread's start routine for the message thread. 
   */
  static void* StartMessageThread(void* This);

  /**
   * P-thread's start routine for the work thread. 
   */
  static void* StartWorkThread(void* This);

  /**
   * Implementation of the message thread.
   */
  void MessageThread();

  /**
   * Implementation of the work thread.
   */
  void WorkThread();

  /**
   * MPI rank and world size.
   */
  MpiInfo mpi;

  /**
   * Receive a message and push it into the message queue (recvQ).
   */
  void RecvWork(const MPI_Status& status, Work* work);

  /**
   * Send a work to all other nodes excluding the node calling this function.
   */
  void SendWorkCopiesToAllOther(Work* work);

  /**
   * An outgoing message queue.
   */
  std::queue<Work*> sendQ;

  /**
   * An incoming message queue.
   */
  std::queue<Work*> recvQ;

  /**
   * Mutex for sendQ.
   */
  pthread_mutex_t sendQ_mutex;

  /**
   * Mutex for recvQ.
   */
  pthread_mutex_t recvQ_mutex;

  /**
   * A condition variable used to make the work thread idle when there is no pending work to do.
   */
  pthread_cond_t recvQ_cond;

  /**
   * A flag indicating if there is pending work or not.
   */
  bool workToDo;

  /**
   * A flag to indicate there is no more work.
   * We don't need a lock since the worker will set it via ThreadJoin().
   */
  bool allWorkDone;

  /**
   * A container holding newly created p-threads.
   */
  std::vector<pthread_t> threads;

  /**
   * A container holding function pointers used to deserialize incoming MPI message.
   */
  std::vector<Work* (*)()> deserializers;
};

}  // namespace unit
}  // namespace render
}  // namespace gvt

#endif
