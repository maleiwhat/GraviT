/* =======================================================================================
   This file is released as part of GraviT - scalable, platform independent ray
   tracing
   tacc.github.io/GraviT

   Copyright 2013-2015 Texas Advanced Computing Center, The University of Texas
   at Austin
   All rights reserved.

   Licensed under the BSD 3-Clause License, (the "License"); you may not use
   this file
   except in compliance with the License.
   A copy of the License is included with this software in the file LICENSE.
   If your copy does not contain the License, you may obtain a copy of the
   License at:

       http://opensource.org/licenses/BSD-3-Clause

   Unless required by applicable law or agreed to in writing, software
   distributed under
   the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY
   KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under
   limitations under the License.

   GraviT is funded in part by the US National Science Foundation under awards
   ACI-1339863,
   ACI-1339881 and ACI-1339840
   =======================================================================================
   */

#ifndef GVT_RENDER_UNIT_WORKER_H
#define GVT_RENDER_UNIT_WORKER_H

#include "gvt/render/unit/Contents.h"

#include <pthread.h>
#include <queue>
#include <memory>
#include <mpi.h>

namespace gvt {
namespace render {
namespace unit {

class Work;
class RayTracer;

struct MpiComm {
  int rank;
  int size;
  MPI_Comm comm;
};

class Worker {
 public:
  Worker(RayTracer* tracer);
  ~Worker();

  int RegisterWork(Work* (*Deserialize)());
  void Start(int argc, char** argv);

  // helper functions for communication
  void Send(Work* work);

  // helper functions
  void SetDone() { allWorkDone = true; }
  RayTracer* GetTracer() { return tracer; }
  int GetRank() { return mpi.rank; }
  int GetMpiRank() { return mpi.rank; }
  int GetMpiSize() { return mpi.size; }
  MPI_Comm GetMpiComm() { return mpi.comm; }

 private:
  enum ThreadId { WORK_THREAD = 0, TRACE_THREAD, NUM_PTHREADS };

  // intializers
  void InitMpi(int argc, char** argv);
  void InitThreads();
  void InitMutexes();

  // p-threads start routines
  static void* StartWorkThread(void* This);
  static void* StartTraceThread(void* This);

  // p-threads
  void WorkThread();
  void TraceThread();

  // main thread for communication
  void Communicator();

  // mpi send/recv
  void RecvWork(const MPI_Status& status, Work* work);
  void SendWork(Work* work);
  void SendWorkAllOther(Work* work);

  void IsendWork(Work* work);

  // work queues
  std::queue<Work*> sendQ;
  std::queue<Work*> recvQ;
  pthread_mutex_t sendQ_mutex;
  pthread_mutex_t recvQ_mutex;

  // flag to indicate there is no more work
  bool allWorkDone;

  // mpi communicator
  MpiComm mpi;

  // pthreads
  std::vector<pthread_t> threads;

  // deserializers used upon receiving a new MPI message
  std::vector<Work* (*)()> deserializers;

  // ray tracer
  RayTracer* tracer;
};

}  // namespace unit
}  // namespace render
}  // namespace gvt

#endif
