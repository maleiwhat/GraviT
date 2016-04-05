#include <iostream>
#include "gvt/core/mpi/Work.h"
#include "gvt/core/mpi/Message.h"

using namespace gvt::core::mpi;

void Work::Serialize(size_t &size, unsigned char *&serialized) {
  size = 0;
  serialized = NULL;
}

Work *Work::Deserialize(size_t size, unsigned char *serialized) {
  if (size != 0) {
    std::cerr << "Work class deserialize called with size != 0\n";
    exit(1);
  }
  return new Work;
}

void Work::Send(int i) {
  Message *m = new Message(this, i);
  m->Enqueue();
}

void Work::Broadcast(bool collective, bool block) {
  Message *m = new Message(this, collective, block);
  m->Enqueue();
  if (block) {
    m->Wait();
  }
}
