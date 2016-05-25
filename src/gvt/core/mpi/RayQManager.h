#pragma once

#include <pthread.h>
#include <queue>

#include "Rays.h"

using namespace std;

namespace gvt {
namespace core {
namespace mpi {

class RayQManager {
public:
  RayQManager();
  ~RayQManager();

  static RayQManager *GetTheRayQManager();
  static RayQManager *theRayQManager;

  void Lock();
  void Unlock();
  void Wait();
  void Signal();

  bool Done() { return done; }
  void Kill();

  void Enqueue(Rays *r);
  Rays *Dequeue();

private:
  pthread_mutex_t lock;
  pthread_cond_t wait;
  pthread_t tid;

  bool done;

  static void *theRayQWorker(void *d);

  queue<Rays *> rayQ;
};
}
}
}
