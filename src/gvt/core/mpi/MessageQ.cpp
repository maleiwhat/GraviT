#include <unistd.h>
#include "iostream"

#include "MessageQ.h"
#include "Message.h"
#include "Application.h"

using namespace std;
using namespace gvt::core::mpi;

void MessageQ::Enqueue(Message *w) {
  pthread_mutex_lock(&lock);
  workq.push(w);
  pthread_cond_signal(&signal);
  pthread_mutex_unlock(&lock);
}

Message *MessageQ::Dequeue() {
  pthread_mutex_lock(&lock);

  while (workq.empty() && running) pthread_cond_wait(&signal, &lock);

  Message *r;
  if (!workq.empty()) {
    r = workq.front();
    workq.pop();
  }

  pthread_mutex_unlock(&lock);
  return r;
}

int MessageQ::IsReady() {
  pthread_mutex_lock(&lock);
  int t = (workq.empty() && running) ? 0 : 1;
  pthread_mutex_unlock(&lock);
  return t;
}

void MessageQ::Kill() {
  pthread_mutex_lock(&lock);
  running = 0;
  pthread_cond_signal(&signal);
  pthread_mutex_unlock(&lock);
}

