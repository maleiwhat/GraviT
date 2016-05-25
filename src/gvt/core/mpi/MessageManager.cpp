#include <mpi.h>
#include <pthread.h>
#include <unistd.h>
#include <iostream>
#include "Application.h"
#include "MessageManager.h"
#include "Message.h"
#include "MessageQ.h"
#include <string>
#include <fstream>
#include <sstream>
#include <memory>

#define LOGGING 0

using namespace gvt::core::mpi;

void *MessageManager::workThread(void *p) {
  MessageManager *theMessageManager = (MessageManager *)p;
  Application *theApplication = Application::GetTheApplication();

  pthread_mutex_lock(&theMessageManager->lock);
  theMessageManager->wait--;
  if (theMessageManager->wait == 0) pthread_cond_signal(&theMessageManager->cond);
  pthread_mutex_unlock(&theMessageManager->lock);

  MessageP m;
  while (theApplication->Running()) {
    Message *m = theMessageManager->GetIncomingMessageQueue()->Dequeue();

    if (!theApplication->Running()) break;

    Work *w = theApplication->Deserialize(m);

    w->Action();
    // TODO: temporary workaround. this should go away (hpark)
    if (!w->deferDeletingThis()) {
      delete w;
    }
  }

  pthread_exit(NULL);
}

void *MessageManager::messageThread(void *p) {
  MessageManager *theMessageManager = (MessageManager *)p;
  Application *theApplication = Application::GetTheApplication();

  int pvd;
  MPI_Init_thread(theApplication->GetPArgC(), theApplication->GetPArgV(), MPI_THREAD_FUNNELED, &pvd);

  MPI_Comm p2p, coll;
  MPI_Comm_dup(MPI_COMM_WORLD, &p2p);
  MPI_Comm_dup(MPI_COMM_WORLD, &coll);
  theMessageManager->setP2PComm(p2p);
  theMessageManager->setCollComm(coll);

  MPI_Comm_rank(MPI_COMM_WORLD, &theMessageManager->mpi_rank);
  MPI_Comm_size(MPI_COMM_WORLD, &theMessageManager->mpi_size);

  pthread_mutex_lock(&theMessageManager->lock);
  theMessageManager->wait--;
  if (theMessageManager->wait == 0) pthread_cond_signal(&theMessageManager->cond);

  pthread_mutex_unlock(&theMessageManager->lock);

  bool kill_me = false;
  while (!kill_me && !theApplication->IsDoneSet()) {
    int read_ready;
    MPI_Status status;

    MPI_Iprobe(MPI_ANY_SOURCE, MPI_ANY_TAG, theMessageManager->getP2PComm(), &read_ready, &status);

    if (read_ready) {
      Message *incoming_message = new Message(status);

      if (incoming_message->IsBroadcast()) theMessageManager->Export(incoming_message);

      if (incoming_message->Communicates()) {
        Work *w = theApplication->Deserialize(incoming_message);
        kill_me = w->Action(theMessageManager->getCollComm());
        delete w;
      } else
        theMessageManager->GetIncomingMessageQueue()->Enqueue(incoming_message);
    }

    if (!kill_me && theMessageManager->GetOutgoingMessageQueue()->IsReady()) {
      Message *outgoing_message = theMessageManager->GetOutgoingMessageQueue()->Dequeue();
      if (outgoing_message) {
        // Send it on
        theMessageManager->Export(outgoing_message);

        // If this message represents a communicating work task, execute its work here in the message thread
        if (outgoing_message->Communicates()) {
          Work *w = theApplication->Deserialize(outgoing_message);
          kill_me = w->Action(theMessageManager->getCollComm());
        }

        if (outgoing_message->blocking) {
          pthread_mutex_lock(&outgoing_message->lock);
          pthread_cond_signal(&outgoing_message->cond);
          pthread_mutex_unlock(&outgoing_message->lock);
        } else
          delete outgoing_message;
      }
    }
  }

  theApplication->Kill();
  theMessageManager->GetOutgoingMessageQueue()->Kill();
  theMessageManager->GetIncomingMessageQueue()->Kill();

  pthread_exit(NULL);
}

MessageManager::MessageManager() {
  Application *theApplication = Application::GetTheApplication();

  pthread_mutex_init(&lock, NULL);
  pthread_cond_init(&cond, NULL);

  pthread_mutex_lock(&lock);

  theIncomingQueue = new MessageQ("incoming");
  theOutgoingQueue = new MessageQ("outgoing");
}

MessageManager::~MessageManager() {
  MPI_Finalize();

  delete theIncomingQueue;
  delete theOutgoingQueue;

  pthread_mutex_unlock(&lock);
}

void MessageManager::Wait() {
  pthread_join(work_tid, NULL);
  pthread_join(message_tid, NULL);
}

void MessageManager::Start() {
  wait = 2;

  if (pthread_create(&message_tid, NULL, messageThread, this)) {
    std::cerr << "Failed to spawn MPI thread\n";
    exit(1);
  }

  if (pthread_create(&work_tid, NULL, workThread, this)) {
    std::cerr << "Failed to spawn work thread\n";
    exit(1);
  }

  while (wait) pthread_cond_wait(&cond, &lock);
}

void MessageManager::SendWork(Work *w, int dest) {
  Message *m = new Message(w, dest);
  GetOutgoingMessageQueue()->Enqueue(m);
}

void MessageManager::BroadcastWork(Work *w, bool block) {
  Message *m = new Message(w, true, block);
  GetOutgoingMessageQueue()->Enqueue(m);

  if (block) m->Wait();
}

void MessageManager::ExportDirect(Message *m) {
  int tag = (random() % MPI_TAG_UB) & 0xfffffe;

#if LOGGING == 1
  std::stringstream s;
  s << Application::GetTheApplication()->GetRank() << " sending " << m->GetHeaderSize() << " to " << m->GetDestination()
    << " tag: " << tag << "\n";
  Application::GetTheApplication()->log(s);
#endif

  int err1 = MPI_Send(m->GetHeader(), m->GetHeaderSize(), MPI_UNSIGNED_CHAR, m->GetDestination(), tag, p2p_comm);

  if (m->HasContent()) {
#if LOGGING == 1
    std::stringstream s;
    s << Application::GetTheApplication()->GetRank() << " sending content " << m->GetSize() << " to "
      << m->GetDestination() << " tag: " << (tag + 1) << "\n";
    Application::GetTheApplication()->log(s);
#endif

    int err2 = MPI_Send(m->GetContent(), m->GetSize(), MPI_UNSIGNED_CHAR, m->GetDestination(), tag + 1, p2p_comm);
  }
}

void MessageManager::Export(Message *m) {
  int rank = Application::GetTheApplication()->GetRank();

  // If its a broadcast message, choose up to two destinations based
  // on the broadcast root, the rank and the size.  Otherwise, just ship it.

  if (m->IsBroadcast()) {
    int size = Application::GetTheApplication()->GetSize();
    int root = m->GetRoot();

    int d = ((size + rank) - m->GetRoot()) % size;

    int l = (2 * d) + 1;
    if (l < size) {
      m->SetDestination((root + l) % size);
      ExportDirect(m);

      if ((l + 1) < size) {
        m->SetDestination((root + l + 1) % size);
        ExportDirect(m);
      }
    }
  } else
    ExportDirect(m);
}
