#pragma once

#include <mpi.h>
#include <stdlib.h>
#include <iostream>
#include <memory>
#include "Message.h"
#include "MessageQ.h"

namespace gvt {
namespace core {
namespace mpi {

class MessageManager {
 public:
  MessageManager();
  ~MessageManager();

  void Wait();

  void Start();
  void WaitForShutdown();

  int GetSize() { return mpi_size; }
  int GetRank() { return mpi_rank; }

  void setP2PComm(MPI_Comm p) { p2p_comm = p; }
  void setCollComm(MPI_Comm c) { coll_comm = c; }

  MPI_Comm getP2PComm() { return p2p_comm; }
  MPI_Comm getCollComm() { return coll_comm; }

  MessageQ *GetIncomingMessageQueue() { return theIncomingQueue; }
  MessageQ *GetOutgoingMessageQueue() { return theOutgoingQueue; }

  void SendWork(Work *w, int dest);
  void BroadcastWork(Work *w, bool blocking);

 private:
  static void *messageThread(void *);
  static void *workThread(void *);

  // This method ships the message to the destination given in the
  // message.

  void ExportDirect(Message *m);

  // This method handles either broadcast or direct messages; if the
  // former, it determines the destinations in the tree distribution
  // pattern and calls ExportDirect to ship it to up to two other
  // destinations; otherwise calls ExportDirect to ship to the
  // message's p2p destination.

  void Export(Message *m);

  MessageQ *theIncomingQueue;
  MessageQ *theOutgoingQueue;

  pthread_mutex_t lock;
  pthread_cond_t cond;

  pthread_t message_tid;
  pthread_t work_tid;

  int wait;
  int mpi_rank;
  int mpi_size;

  MPI_Comm p2p_comm, coll_comm;
};
}
}
}
