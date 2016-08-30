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
// Communicator.cpp
//
#include "gvt/render/unit/Communicator.h"

// works
#include "gvt/render/unit/DomainWorks.h"
#include "gvt/render/unit/Work.h"

// tracers
#include "gvt/render/unit/RayTracer.h"

// TPC voter
#include "gvt/render/unit/Voter.h"

#include <mpi.h>
#include <iostream>

namespace gvt {
namespace render {
namespace unit {

Communicator::Communicator(const MpiInfo &mpi, Worker *worker) : mpi(mpi), worker(worker), allWorkDone(false) {
  InitThreads();
}

int Communicator::RegisterWork(Work *(*Deserialize)()) {
  int tag = deserializers.size();
  deserializers.push_back(Deserialize);
  return tag;
}

void Communicator::InitThreads() {
  InitMutexes();

  threads.resize(NUM_PTHREADS);

  int error = pthread_create(&threads[MESSAGE_THREAD], NULL, &Communicator::StartMessageThread, this);
  if (error) {
    std::cout << "error " << error << " failed to create message thread.\n";
    exit(error);
  }
}

void Communicator::InitMutexes() { pthread_mutex_init(&sendQ_mutex, NULL); }

void *Communicator::StartMessageThread(void *This) {
  Communicator *worker = static_cast<Communicator *>(This);
  worker->MessageThread();
  return NULL;
}

void Communicator::MessageThread() {
  worker->WaitTracerReady();

  while (!allWorkDone) {
    // serve incoming message
    int flag;
    MPI_Status mpi_status;
    int mpi_error = MPI_Iprobe(MPI_ANY_SOURCE, MPI_ANY_TAG, MPI_COMM_WORLD, &flag, &mpi_status);
#ifdef DEBUG_COMM
    if (mpi_error != MPI_SUCCESS) {
      int len, eclass;
      char estring[MPI_MAX_ERROR_STRING];
      MPI_Error_class(mpi_error, &eclass);
      MPI_Error_string(mpi_error, estring, &len);
      printf("error %d: %s\n", eclass, estring);
      exit(1);
    }
#endif

    if (flag) {
      assert(mpi_status.MPI_TAG >= 0 && mpi_status.MPI_TAG < deserializers.size());
#ifdef DEBUG_COMM
      if (!(mpi_status.MPI_TAG >= 0 && mpi_status.MPI_TAG < deserializers.size())) {
        printf("error mpi tag %d out of bound\n", mpi_status.MPI_TAG);
        exit(1);
      }
#endif
      Work *incoming_work = deserializers[mpi_status.MPI_TAG]();
      RecvWork(mpi_status, incoming_work);
    }

    // serve outgoing message
    pthread_mutex_lock(&sendQ_mutex);
    while (!sendQ.empty()) {
      Work *outgoing_work = sendQ.front();
      if (!outgoing_work->IsSent()) break;
#ifdef DEBUG_COMM
      int tag = outgoing_work->GetTag();
      std::size_t size = outgoing_work->GetSize();
      std::size_t qsize = sendQ.size();
      std::cout << "[COMM SENDQ] rank " << mpi.rank << " popped work tag " << tag << "(size: " << size
                << ") from sendQ (size " << qsize << ")" << std::endl;
#endif
      sendQ.pop();
      delete outgoing_work;
    }
    pthread_mutex_unlock(&sendQ_mutex);
  }
#ifdef DEBUG_COMM
  std::cout << "[COMM] rank " << mpi.rank << " messageThread broke out of the loop." << std::endl;
#endif
}

void Communicator::Quit() {
#ifdef DEBUG_COMM
  std::cout << "[COMM QUIT] rank " << mpi.rank << " seting allWorkDOne = 1 " << __PRETTY_FUNCTION__ << std::endl;
#endif
  allWorkDone = true;
  for (std::size_t i = 0; i < threads.size(); ++i) {
    pthread_join(threads[i], NULL);
#ifdef DEBUG_COMM
    std::cout << "[COMM] rank " << mpi.rank << " thread " << i << " / " << threads.size() << " joined." << std::endl;
#endif
  }
  // we don't need lock/unlock but just in case
  pthread_mutex_lock(&sendQ_mutex);
  if (!sendQ.empty()) {
    std::cout << "error rank " << mpi.rank << " send queue not empty. size " << sendQ.size() << "\n";
    exit(1);
  }
  pthread_mutex_unlock(&sendQ_mutex);
}

void Communicator::RecvWork(const MPI_Status &status, Work *work) {
  int count = 0;
  MPI_Get_count(&status, MPI_UNSIGNED_CHAR, &count);

  assert(count > 0);

  work->Allocate(count);
#ifdef DEBUG_COMM
  printf("[COMM MPI_Recv] rank %d buf %p count %d src %d tag %d\n", mpi.rank, work->GetBufferPtr<unsigned char>(),
         count, status.MPI_SOURCE, status.MPI_TAG);
#endif
  MPI_Status status_out;
  MPI_Recv(work->GetBufferPtr<unsigned char>(), count, MPI_UNSIGNED_CHAR, status.MPI_SOURCE, status.MPI_TAG,
           MPI_COMM_WORLD, &status_out);

#ifdef DEBUG_COMM
  int count_recved = 0;
  MPI_Get_count(&status_out, MPI_UNSIGNED_CHAR, &count_recved);
  if (count_recved != count) {
    std::cout << "error count mismatch!\n";
  }
  assert(count_recved == count);
#endif

  HandleReceivedMessage(work);
}

void Communicator::Send(int dest, Work *work) {
  int count = work->GetSize();

  assert(count > 0);
  assert(work->GetContents());

#ifdef DEBUG_COMM
  printf("[MPI_Isend] rank %d buf %p count %d dest %d tag %d rqst %p\n", mpi.rank, work->GetBufferPtr<unsigned char>(),
         count, dest, work->GetTag(), &work->GetMpiRequest());
#endif

  MPI_Isend(work->GetBufferPtr<unsigned char>(), count, MPI_UNSIGNED_CHAR, dest, work->GetTag(), MPI_COMM_WORLD,
            &work->GetMpiRequest());

#ifdef DEBUG_COMM
  printf("MPI_Isend done\n");
#endif

  pthread_mutex_lock(&sendQ_mutex);
  sendQ.push(work);
  pthread_mutex_unlock(&sendQ_mutex);
}

void Communicator::SendAll(Work *work) {
  SendWorkCopiesToAllOther(work);
  HandleReceivedMessage(work);
}

void Communicator::SendAllOther(Work *work) {
  SendWorkCopiesToAllOther(work);

  // we don't need the original copy any more
  delete work;
}

void Communicator::SendWorkCopiesToAllOther(Work *work) {
  pthread_mutex_lock(&sendQ_mutex);
  for (int dest = 0; dest < mpi.size; ++dest) {
    if (dest != mpi.rank) {
      Work *work_copy = deserializers[work->GetTag()]();
      work_copy->Clone(work);

#ifdef DEBUG_COMM
      printf("[MPI_Isend] rank %d buf %p count %d dest %d tag %d rqst %p\n", mpi.rank,
             work_copy->GetBufferPtr<unsigned char>(), work_copy->GetSize(), dest, work_copy->GetTag(),
             &work_copy->GetMpiRequest());
#endif

      MPI_Isend(work_copy->GetBufferPtr<unsigned char>(), work_copy->GetSize(), MPI_UNSIGNED_CHAR, dest,
                work_copy->GetTag(), MPI_COMM_WORLD, &work_copy->GetMpiRequest());
#ifdef DEBUG_COMM
      printf("MPI_Isend done\n");
#endif
      sendQ.push(work_copy);
    }
  }
  pthread_mutex_unlock(&sendQ_mutex);
}

inline void Communicator::HandleReceivedMessage(Work *work) {
  bool delete_this = work->Action(worker);
  if (delete_this) delete work;
}

} // namespace unit
} // namespace render
} // namespace gvt

