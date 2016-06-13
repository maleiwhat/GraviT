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

#include "gvt/render/unit/Worker.h"

// works
#include "gvt/render/unit/DomainWorks.h"
#include "gvt/render/unit/Work.h"

// tracers
#include "gvt/render/unit/RayTracer.h"

// TPC voter
#include "gvt/render/unit/TpcVoter.h"

#include <mpi.h>
#include <iostream>

namespace gvt {
namespace render {
namespace unit {

Worker::Worker(RayTracer* tracer)
    : tracer(tracer), allWorkDone(false), voter(NULL) {
  // initialization
  InitMpi();
  InitThreads();

  // create a voter
  if (mpi.size > 1) voter = new TpcVoter(mpi.size, mpi.rank, *tracer, this);
}

Worker::~Worker() {
  if (voter) delete voter;
}

int Worker::RegisterWork(Work* (*Deserialize)()) {
  int tag = deserializers.size();
  deserializers.push_back(Deserialize);
  return tag;
}

void Worker::Send(Work* work) {
  pthread_mutex_lock(&sendQ_mutex);
  sendQ.push(work);
  pthread_mutex_unlock(&sendQ_mutex);
}

void Worker::InitMpi() {
  // warning: this should be in the beginning of main()
  // int provided;
  // MPI_Init_thread(&argc, &argv, MPI_THREAD_SINGLE, &provided);
  // if (provided != MPI_THREAD_SINGLE) {
  //   std::cout << "error mpi_thread_single not available\n";
  //   exit(1);
  // }

  // warning: MPI_Comm_dup is causing some problem.
  // directly use MPI_COMM_WORLD for now.
  // int dup_error = MPI_Comm_dup(MPI_COMM_WORLD, &mpi.comm);
  // if (dup_error != MPI_SUCCESS) {
  //   std::cout << "error MPI_Comm_dup not successful\n";
  //   exit(1);
  // }
  mpi.comm = MPI_COMM_WORLD;
  MPI_Comm_rank(mpi.comm, &mpi.rank);
  MPI_Comm_size(mpi.comm, &mpi.size);
}

void Worker::InitThreads() {
  InitMutexes();

  threads.resize(NUM_PTHREADS);
  int error = pthread_create(&threads[WORK_THREAD], NULL,
                             &Worker::StartWorkThread, this);
  if (error) {
    std::cout << "error " << error << " failed to create worker thread.\n";
    exit(error);
  }

  error = pthread_create(&threads[TRACE_THREAD], NULL,
                         &Worker::StartTraceThread, this);
  if (error) {
    std::cout << "error " << error << " failed to create trace thread.\n";
    exit(error);
  }
}

void Worker::InitMutexes() {
  pthread_mutex_init(&sendQ_mutex, NULL);
  pthread_mutex_init(&recvQ_mutex, NULL);
}

void* Worker::StartWorkThread(void* This) {
  Worker* worker = static_cast<Worker*>(This);
  worker->WorkThread();
  return NULL;
}

void* Worker::StartTraceThread(void* This) {
  Worker* worker = static_cast<Worker*>(This);
  worker->TraceThread();
  return NULL;
}

inline void Worker::WorkThread() {
  while (!allWorkDone) {
    Work* work = NULL;

    pthread_mutex_lock(&recvQ_mutex);
    if (!recvQ.empty()) {
      work = recvQ.front();
      recvQ.pop();
    }
    pthread_mutex_unlock(&recvQ_mutex);

    if (work) {
      work->Action(this);
      delete work;
    }
  }
}

inline void Worker::TraceThread() {
  tracer->Trace(this);
}

void Worker::Communicator() {
  bool done = false;
  MPI_Status mpi_status;

  while (!allWorkDone) {
    // serve incoming message
    int flag;
    MPI_Iprobe(MPI_ANY_SOURCE, MPI_ANY_TAG, mpi.comm, &flag, &mpi_status);

    if (flag) {
      Work* incoming_work = deserializers[mpi_status.MPI_TAG]();
      RecvWork(mpi_status, incoming_work);
    }

    // serve outgoing message
    Work* outgoing_work = NULL;
    pthread_mutex_lock(&sendQ_mutex);
    if (!sendQ.empty()) {
      outgoing_work = sendQ.front();
      sendQ.pop();
    }
    pthread_mutex_unlock(&sendQ_mutex);

    if (outgoing_work) SendWork(outgoing_work);
  }

  // join all p-threads
  for (std::size_t i = 0; i < threads.size(); ++i) {
    pthread_join(threads[i], NULL);
#ifndef NDEBUG
    std::cout << "rank " << mpi.rank << "thread " << i << " / "
              << threads.size() << " joined.\n";
#endif
  }

  // error checking code
  // we don't need lock/unlock but just in case
  pthread_mutex_lock(&sendQ_mutex);
  if (!sendQ.empty()) {
    std::cout << "error rank " << mpi.rank << " send queue not empty. size "
              << sendQ.size() << "\n";
    exit(1);
  }
  pthread_mutex_unlock(&sendQ_mutex);

  pthread_mutex_lock(&recvQ_mutex);
  if (!recvQ.empty()) {
    std::cout << "error rank " << mpi.rank << " recv queue not empty. size "
              << recvQ.size() << "\n";
    exit(1);
  }
  pthread_mutex_unlock(&recvQ_mutex);
}

void Worker::RecvWork(const MPI_Status& status, Work* work) {
  int count = 0;
  MPI_Get_count(&status, MPI_UNSIGNED_CHAR, &count);

#ifndef NDEBUG
  std::cout << "count " << count << " " << __PRETTY_FUNCTION__ << "\n";
#endif

  // TODO
  if (count < 1) {
    std::cout << "error unable to receive " << count << " bytes. tag "
              << status.MPI_TAG << " source " << status.MPI_SOURCE << "\n";
    exit(1);
  }

  work->Allocate(count);
  MPI_Status status_out;
  MPI_Recv(work->GetBuffer(), count, MPI_UNSIGNED_CHAR, status.MPI_SOURCE,
           status.MPI_TAG, mpi.comm, &status_out);

  int count_recved = 0;
  MPI_Get_count(&status_out, MPI_UNSIGNED_CHAR, &count_recved);
  if (count_recved != count) {
    std::cout << "error count mismatch!\n";
    exit(1);
  }

  pthread_mutex_lock(&recvQ_mutex);
  recvQ.push(work);
#ifndef NDEBUG
  std::cout << "rank " << mpi.rank << "pushed work tag " << work->GetTag()
            << " to recvQ (size " << recvQ.size() << ")" << std::endl;
#endif
  pthread_mutex_unlock(&recvQ_mutex);
}

void Worker::SendWork(Work* work) {
  int comm_type = work->GetCommType();
  if (comm_type == Work::SEND_ALL_OTHER) {
    SendWorkAllOther(work);
    delete work;
  } else if (comm_type == Work::SEND_ALL) {
    SendWorkAllOther(work);

    pthread_mutex_lock(&recvQ_mutex);
    recvQ.push(work);
    pthread_mutex_unlock(&recvQ_mutex);
  } else {  // P2P
    // TODO
    int count = work->GetSize();
    if (count < 1) {
      std::cout << "error unable to send " << count << " bytes.\n";
      exit(1);
    }
    MPI_Send(work->GetBuffer(), count, MPI_UNSIGNED_CHAR,
             work->GetDestination(), work->GetTag(), mpi.comm);
    delete work;
  }
}

void Worker::SendWorkAllOther(Work* work) {
  for (int dest = 0; dest < mpi.size; ++dest) {
    if (dest != mpi.rank) {
      MPI_Send(work->GetBuffer(), work->GetSize(), MPI_UNSIGNED_CHAR, dest,
               work->GetTag(), mpi.comm);
    }
  }
}

void Worker::IsendWork(Work* work) {
  // TBD
}

}  // namespace unit
}  // namespace render
}  // namespace gvt

