#include "gvt/render/unit/Communicator.h"

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

// Communicator::Communicator() { Initialize(); }

// Communicator::Communicator(int* argc, char*** argv, Worker* worker)
//     : argcp(argc), argvp(argv), worker(worker), allWorkDone(false) {
Communicator::Communicator(const MpiInfo& mpi, Worker* worker)
    : mpi(mpi), worker(worker), allWorkDone(false) {
  InitThreads();
}

// void Communicator::Initialize() {
//   allWorkDone = false;
//   voter = tracer->GetVoter();
//
//   // initialization
//   InitMpi();
//   InitThreads();
//
//   // // create a voter
//   // if (mpi.size > 1) voter = new TpcVoter(mpi.size, mpi.rank, *tracer,
//   this);
// }

int Communicator::RegisterWork(Work* (*Deserialize)()) {
  int tag = deserializers.size();
  deserializers.push_back(Deserialize);
  return tag;
}

// void Communicator::InitMpi() {
//   // warning: this should be in the beginning of main()
//   // int provided;
//   // MPI_Init_thread(&argc, &argv, MPI_THREAD_SINGLE, &provided);
//   // if (provided != MPI_THREAD_SINGLE) {
//   //   std::cout << "error mpi_thread_single not available\n";
//   //   exit(1);
//   // }
//
//   // warning: MPI_Comm_dup is causing some problem.
//   // directly use MPI_COMM_WORLD for now.
//   // int dup_error = MPI_Comm_dup(MPI_COMM_WORLD, &mpi.comm);
//   // if (dup_error != MPI_SUCCESS) {
//   //   std::cout << "error MPI_Comm_dup not successful\n";
//   //   exit(1);
//   // }
//   mpi.comm = MPI_COMM_WORLD;
//   MPI_Comm_rank(mpi.comm, &mpi.rank);
//   MPI_Comm_size(mpi.comm, &mpi.size);
// }

void Communicator::InitThreads() {
  InitMutexes();

  threads.resize(NUM_PTHREADS);

  int error = pthread_create(&threads[WORK_THREAD], NULL,
                         &Communicator::StartWorkThread, this);
  if (error) {
    std::cout << "error " << error << " failed to create worker thread.\n";
    exit(error);
  }

  error = pthread_create(&threads[MESSAGE_THREAD], NULL,
                         &Communicator::StartMessageThread, this);
  if (error) {
    std::cout << "error " << error << " failed to create message thread.\n";
    exit(error);
  }
}

void Communicator::InitMutexes() {
  pthread_mutex_init(&sendQ_mutex, NULL);
  pthread_mutex_init(&recvQ_mutex, NULL);
}

void* Communicator::StartWorkThread(void* This) {
  Communicator* worker = static_cast<Communicator*>(This);
  worker->WorkThread();
  return NULL;
}

void* Communicator::StartMessageThread(void* This) {
  Communicator* worker = static_cast<Communicator*>(This);
  worker->MessageThread();
  return NULL;
}

inline void Communicator::WorkThread() {
  while (!allWorkDone) {
    Work* work = NULL;

    pthread_mutex_lock(&recvQ_mutex);
    if (!recvQ.empty()) {
      work = recvQ.front();
      recvQ.pop();
      assert(work);
#ifndef NDEBUG
      std::cout << "[COMM RECVQ] rank " << mpi.rank << " popped work tag "
                << work->GetTag() << " from recvQ (size " << recvQ.size() << ")"
                << std::endl;
#endif
    }
    pthread_mutex_unlock(&recvQ_mutex);

    if (work) {
      bool delete_this = work->Action(worker);
      if (delete_this) delete work;
    }
  }
#ifndef NDEBUG
  std::cout << "[COMM] rank " << mpi.rank
            << " workThread broke out of the loop." << std::endl;
#endif
}

void Communicator::MessageThread() {
  // int pvd;
  // MPI_Init_thread(argcp, argvp, MPI_THREAD_MULTIPLE, &pvd);
  // if ((pvd != MPI_THREAD_MULTIPLE)) {
  //   std::cerr << "error: mpi_thread_multiple not available\n";
  //   exit(1);
  // }
  // // MPI_Init(argcp, argvp);

  // MPI_Comm_rank(MPI_COMM_WORLD, &mpi.rank);
  // MPI_Comm_size(MPI_COMM_WORLD, &mpi.size);
  
  // worker->SignalMpiReady();
  worker->WaitTracerReady();

  while (!allWorkDone) {
    // serve incoming message
    int flag;
    MPI_Status mpi_status;
    MPI_Iprobe(MPI_ANY_SOURCE, MPI_ANY_TAG, MPI_COMM_WORLD, &flag, &mpi_status);

    if (flag) {
      assert(mpi_status.MPI_TAG >= 0 &&
             mpi_status.MPI_TAG < deserializers.size());
      Work* incoming_work = deserializers[mpi_status.MPI_TAG]();
      RecvWork(mpi_status, incoming_work);
    }

    // serve outgoing message
    pthread_mutex_lock(&sendQ_mutex);
    while (!sendQ.empty()) {
      Work* outgoing_work = sendQ.front();
      if (!outgoing_work->IsSent()) break;
#ifndef NDEBUG
      std::cout << "[COMM SENDQ] rank " << mpi.rank << " popped work tag "
                << outgoing_work->GetTag()
                << "(size: " << outgoing_work->GetSize()
                << ") from sendQ (size " << sendQ.size() << ")" << std::endl;
#endif
      sendQ.pop();
      delete outgoing_work;
    }
    pthread_mutex_unlock(&sendQ_mutex);
  }
#ifndef NDEBUG
  std::cout << "[COMM] rank " << mpi.rank
            << " messageThread broke out of the loop." << std::endl;
#endif
}

void Communicator::Quit() {
#ifndef NDEBUG
  std::cout << "[COMM QUIT] rank " << mpi.rank << " seting allWorkDOne = 1 "
            << __PRETTY_FUNCTION__ << std::endl;
#endif
  allWorkDone = true;
  for (std::size_t i = 0; i < threads.size(); ++i) {
    pthread_join(threads[i], NULL);
#ifndef NDEBUG
    std::cout << "[COMM] rank " << mpi.rank << " thread " << i << " / "
              << threads.size() << " joined." << std::endl;
#endif
  }
  // we don't need lock/unlock but just in case
  pthread_mutex_lock(&sendQ_mutex);
  if (!sendQ.empty()) {
    // Work* work = sendQ.front();
    // sendQ.pop();
    // delete work;
    std::cout << "error rank " << mpi.rank << " send queue not empty. size "
              << sendQ.size() << "\n";
    exit(1);
  }
  pthread_mutex_unlock(&sendQ_mutex);

  // error checking code
  pthread_mutex_lock(&recvQ_mutex);
  if (!recvQ.empty()) {
    std::cout << "error rank " << mpi.rank << " recv queue not empty. size "
              << recvQ.size() << "\n";
    exit(1);
  }
  pthread_mutex_unlock(&recvQ_mutex);
}

void Communicator::RecvWork(const MPI_Status& status, Work* work) {
  int count = 0;
  MPI_Get_count(&status, MPI_UNSIGNED_CHAR, &count);

// #ifndef NDEBUG
//   std::cout << "rank " << mpi.rank << " count " << count << " "
//             << __PRETTY_FUNCTION__ << "\n";
// #endif

  assert(count > 0);

  work->Allocate(count);
#ifndef NDEBUG
  printf("[COMM MPI_Recv] buf %p count %d src %d tag %d\n", work->GetBuffer(),
         count, status.MPI_SOURCE, status.MPI_TAG);
#endif
  MPI_Status status_out;
  MPI_Recv(work->GetBuffer(), count, MPI_UNSIGNED_CHAR, status.MPI_SOURCE,
           status.MPI_TAG, MPI_COMM_WORLD, &status_out);

#ifndef NDEBUG
  int count_recved = 0;
  MPI_Get_count(&status_out, MPI_UNSIGNED_CHAR, &count_recved);
  if (count_recved != count) {
    std::cout << "error count mismatch!\n";
  }
  assert(count_recved == count);
#endif

  pthread_mutex_lock(&recvQ_mutex);
  recvQ.push(work);
#ifndef NDEBUG
  std::cout << "[COMM RECVQ] rank " << mpi.rank << " pushed work tag "
            << work->GetTag() << " to recvQ (size " << recvQ.size() << ")"
            << std::endl;
#endif
  pthread_mutex_unlock(&recvQ_mutex);
}

void Communicator::Send(Work* work) {
  int comm_type = work->GetCommType();
  if (comm_type == Work::SEND_ALL_OTHER) {
    SendWorkAllOther(work);
  } else if (comm_type == Work::SEND_ALL) {
    work->Action(worker);  // TODO: inefficient (i.e. have to defer sendWOrkAllOther)
    SendWorkAllOther(work);

    // pthread_mutex_lock(&recvQ_mutex);
    // recvQ.push(work);
    // pthread_mutex_unlock(&recvQ_mutex);
  } else {  // P2P
    int count = work->GetSize();

    assert(count > 0);
    assert(work->GetBuffer());

    MPI_Isend(work->GetBuffer(), count, MPI_UNSIGNED_CHAR, work->GetDestination(),
              work->GetTag(), MPI_COMM_WORLD, &work->GetMpiRequest());

    pthread_mutex_lock(&sendQ_mutex);
    sendQ.push(work);
    pthread_mutex_unlock(&sendQ_mutex);
  }
}

void Communicator::SendWorkAllOther(Work* work) {
  pthread_mutex_lock(&sendQ_mutex);
  for (int dest = 0; dest < mpi.size; ++dest) {
    if (dest != mpi.rank) {
      MPI_Isend(work->GetBuffer(), work->GetSize(), MPI_UNSIGNED_CHAR, dest,
               work->GetTag(), MPI_COMM_WORLD, &work->GetMpiRequest());
      sendQ.push(work);
    }
  }
  pthread_mutex_unlock(&sendQ_mutex);
}

void Communicator::IsendWork(Work* work) {
  // TBD
}

}  // namespace unit
}  // namespace render
}  // namespace gvt

