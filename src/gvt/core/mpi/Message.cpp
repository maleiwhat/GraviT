#include <mpi.h>
#include <pthread.h>
#include <unistd.h>
#include <iostream>
#include "Message.h"
#include "Application.h"
#include <string>
#include <fstream>
#include <sstream>
#include <memory>

#define LOGGING 0

using namespace gvt::core::mpi;

Message::Message(Work *w, bool c, bool b) {
  header.type = w->GetType();

  blocking = b;
  if (blocking) {
    pthread_mutex_init(&lock, NULL);
    pthread_cond_init(&cond, NULL);
    pthread_mutex_lock(&lock);
  }

  header.communicates = c;
  header.broadcast_root = Application::GetTheApplication()->GetRank();

  content = w->get_pointer();
  header.SetHasContent(content->get_size() != 0);

#if LOGGING == 1
  std::stringstream s;
  s << id << (header.communicates ? "+c\n" : "+n\n");
  Application::GetTheApplication()->log(s);
#endif
}

Message::Message(Work *w, int i) {
  header.type = w->GetType();

  blocking = false;

  header.broadcast_root = -1;
  header.communicates = false;

  destination = i;
  content = w->get_pointer();
  header.SetHasContent(content->get_size() != 0);

#if LOGGING == 1
  std::stringstream s;
  s << id << "+n\n";
  Application::GetTheApplication()->log(s);
#endif
}

Message::Message(MPI_Status &status) {
  Application *theApplication = Application::GetTheApplication();
  MessageManager *theMessageManager = theApplication->GetTheMessageManager();

  int count;
  MPI_Get_count(&status, MPI_UNSIGNED_CHAR, &count);

  if (count != sizeof(header)) std::cerr << "ERROR 1\n";

  if (status.MPI_TAG & 0x1) {
    std::cerr << "error - odd tag in Message::Message\n";
    exit(1);
  }

#if LOGGING == 1
  std::stringstream ss;
  ss << "  receiving " << sizeof(header) << " at tag " << status.MPI_TAG << "\n";
  theApplication->log(ss);
#endif

  MPI_Status s0;
  MPI_Recv((unsigned char *)&header, sizeof(header), MPI_UNSIGNED_CHAR, status.MPI_SOURCE, status.MPI_TAG,
           theMessageManager->getP2PComm(), &s0);

  if (header.HasContent()) {
    MPI_Status s;
    MPI_Probe(status.MPI_SOURCE, status.MPI_TAG + 1, theMessageManager->getP2PComm(), &s);
    MPI_Get_count(&s, MPI_UNSIGNED_CHAR, &count);

#if LOGGING == 1
    std::stringstream ss;
    ss << "  receiving " << count << " at tag " << s.MPI_TAG << "\n";
    theApplication->log(ss);
#endif

    content = smem::New(count);
    MPI_Recv(content->get(), content->get_size(), MPI_UNSIGNED_CHAR, s.MPI_SOURCE, s.MPI_TAG,
             theMessageManager->getP2PComm(), &s);
  }
}

Message::Message() {

#if LOGGING == 1
  std::stringstream s;
  s << "pending...  " << id << "\n";
  Application::GetTheApplication()->log(s);
#endif
}

Message::~Message() {
#if LOGGING == 1
  std::stringstream s;
  s << id << (header.communicates ? "-c\n" : "-n\n");
  Application::GetTheApplication()->log(s);
#endif
}

void Message::Wait() {
  if (!blocking)
    std::cerr << "Error... waiting on non-blocking message?\n";
  else
    pthread_cond_wait(&cond, &lock);
}

bool Message::Communicates() { return header.communicates; }
