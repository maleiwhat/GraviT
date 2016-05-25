#pragma once

#include <iostream>
#include <queue>
#include <pthread.h>
#include "Message.h"

namespace gvt {
namespace core {
namespace mpi {

class MessageQ {
public:
  MessageQ(const char *n) : name(n) {
    pthread_mutex_init(&lock, NULL);
    pthread_cond_init(&signal, NULL);
    running = true;
  }

  ~MessageQ() {
    running = false;
    pthread_cond_broadcast(&signal);
  }

  void Kill();

  void Enqueue(Message *w);
  Message *Dequeue();
  int IsReady();

private:
  const char *name;

  pthread_mutex_t lock;
  pthread_cond_t signal;
  bool running;

  queue<Message *> workq;
};

}
}
}
