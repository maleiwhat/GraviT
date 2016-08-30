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
// Communicator.h
//

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
class Voter;

class Communicator {
public:
  Communicator(const MpiInfo &mpi, Worker *worker);

  /**
   * Register a function pointer to message deserializer defined in the work class.
   */
  int RegisterWork(Work *(*Deserialize)());

  /**
   * Kill all the pthreads and check if all the message queues are empty or not.
   */
  void Quit();

  /**
   * Send a work to node specified by dest.
   */
  void Send(int dest, Work *work);

  /**
   * Send a work to all nodes including the node that calls this function.
   */
  void SendAll(Work *work);

  /**
   * Send a work to all the rest nodes excluding the node that class this function.
   */
  void SendAllOther(Work *work);

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

private:
  /**
   * Unique IDs associated with created p-threads for MPI communication.
   */
  enum ThreadId {
    MESSAGE_THREAD = 0,
    NUM_PTHREADS
  };

  /**
   * A pointer to the worker object. This is used to call the Action function in the work class.
   */
  Worker *worker;

  /**
   * Creates p-threads for MPI commnication.
   */
  void InitThreads();

  /**
   * Initializes required mutexes.
   */
  void InitMutexes();

  /**
   * Handle received message.
   */
  void HandleReceivedMessage(Work *work);

  /**
   * P-thread's start routine for the message thread.
   */
  static void *StartMessageThread(void *This);

  /**
   * Implementation of the message thread.
   */
  void MessageThread();

  /**
   * MPI rank and world size.
   */
  MpiInfo mpi;

  /**
   * Receive a message and push it into the message queue (recvQ).
   */
  void RecvWork(const MPI_Status &status, Work *work);

  /**
   * Send a work to all other nodes excluding the node calling this function.
   */
  void SendWorkCopiesToAllOther(Work *work);

  /**
   * An outgoing message queue.
   */
  std::queue<Work *> sendQ;

  /**
   * Mutex for sendQ.
   */
  pthread_mutex_t sendQ_mutex;

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
  std::vector<Work *(*)()> deserializers;
};

} // namespace unit
} // namespace render
} // namespace gvt

#endif
