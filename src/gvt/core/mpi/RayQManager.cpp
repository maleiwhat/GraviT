#include <iostream>
#include <pthread.h>

#include "Application.h"
#include "RayQManager.h"
#include "Rays.h"

using namespace std;
using namespace gvt::core::mpi;

RayQManager *RayQManager::theRayQManager;
RayQManager *RayQManager::GetTheRayQManager() { return RayQManager::theRayQManager; }

void *RayQManager::theRayQWorker(void *d) {
  RayQManager *theRayQManager = (RayQManager *)d;

  Rays *r;
  while ((r = theRayQManager->Dequeue()) != NULL) {
    cerr << "p";
    // process r
    delete r;
  };

  theRayQManager->Unlock();
  pthread_exit(0);
}

RayQManager::RayQManager() {
  RayQManager::theRayQManager = this;

  pthread_mutex_init(&lock, NULL);
  pthread_cond_init(&wait, NULL);

  done = false;

  pthread_create(&tid, NULL, RayQManager::theRayQWorker, this);
}

void RayQManager::Kill() {
  Lock();
  done = true;
  Signal();
  Unlock();
}

RayQManager::~RayQManager() {
  Kill();
  pthread_join(tid, NULL);
}

void RayQManager::Lock() { pthread_mutex_lock(&lock); }

void RayQManager::Unlock() { pthread_mutex_unlock(&lock); }

void RayQManager::Wait() { pthread_cond_wait(&wait, &lock); }

void RayQManager::Signal() { pthread_cond_signal(&wait); }

Rays *RayQManager::Dequeue() {
  Rays *r = NULL;

  Lock();
  while (!done && rayQ.empty()) Wait();

  if (!rayQ.empty()) {
    r = rayQ.front();
    rayQ.pop();
  }

  Unlock();
  return r;
}

void RayQManager::Enqueue(Rays *r) {
  Lock();
  rayQ.push(r);
  Signal();
  Unlock();
}
