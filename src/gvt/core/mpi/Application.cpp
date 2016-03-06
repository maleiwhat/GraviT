/* =======================================================================================
   This file is released as part of GraviT - scalable, platform independent ray
   tracing
   tacc.github.io/GraviT

   Copyright 2013-2015 Texas Advanced Computing Center, The University of Texas
   at Austin
   All rights reserved.

   Licensed under the BSD 3-Clause License, (the "License"); you may not use
   this file
   except in compliance with the License.
   A copy of the License is included with this software in the file LICENSE.
   If your copy does not contain the License, you may obtain a copy of the
   License at:

       http://opensource.org/licenses/BSD-3-Clause

   Unless required by applicable law or agreed to in writing, software
   distributed under
   the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY
   KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under
   limitations under the License.

   GraviT is funded in part by the US National Science Foundation under awards
   ACI-1339863,
   ACI-1339881 and ACI-1339840
   =======================================================================================
   */

//
// Applicatoin.cpp
//

#ifndef GVT_CORE_MPI_APPLICATION_CPP
#define GVT_CORE_MPI_APPLICATION_CPP

#include <mpi.h>
#include <unistd.h>
#include <iostream>
#include "gvt/core/mpi/Application.h"
#include "gvt/render/unit/TileWork.h"
#include "gvt/render/unit/ImageTileWork.h"
#include "gvt/render/unit/DomainTileWork.h"

// #define DEBUG_APP
// #define DEBUG_APP_LOCK
#define DEBUG_DISABLE_WORKTHREAD 0

using namespace gvt::core::mpi;
using namespace gvt::render::unit;

WORK_CLASS(Quit)

Application *theApplication = NULL;

Application *Application::GetApplication() { return theApplication; }

Application::Application(int *a, char ***b) {
  theApplication = this;
  application_done = false;

  argcp = a;
  argvp = b;

  theMessageManager.Initialize();

  pthread_mutex_init(&lock, NULL);
  pthread_cond_init(&cond, NULL);

  pthread_mutex_lock(&lock);
#ifdef DEBUG_APP_LOCK
  printf("Rank %d: Application::Application lock\n", GetRank());
#endif
  theIncomingQueue = new MessageQ("incoming");
  theOutgoingQueue = new MessageQ("outgoing");

  Quit::Register();
  // GDA pthread_mutex_unlock(&lock);
#ifdef DEBUG_APP_LOCK
  printf("Rank %d: Application::Application unlock\n", GetRank());
#endif

  // theApplication = this;
#ifdef DEBUG_APP
  printf("Rank %d: Application::Application: done\n", Application::GetApplication()->GetRank());
#endif
}

Application::~Application() {
  Wait();
  // GDA
  pthread_mutex_unlock(&lock);
  delete theIncomingQueue;
  delete theOutgoingQueue;
}

void Application::QuitApplication() {
  Quit quit;
  quit.Broadcast(true, true);
}

void Application::Start() {
  GetMessageManager()->Start();

  if (pthread_create(&work_thread_tid, NULL, workThread, this)) {
    std::cerr << "Failed to spawn work thread\n";
    exit(1);
  }

  // Wait for ping from worker thread
#ifdef DEBUG_APP_LOCK
  printf("Rank %d: Application::Start. blocked on cond\n", Application::GetApplication()->GetRank());
#endif
  pthread_cond_wait(&cond, &lock);
  // pthread_mutex_unlock(&lock);
#ifdef DEBUG_APP_LOCK
  printf("Rank %d: Application::Start. cond unblocked\n", Application::GetApplication()->GetRank());
#endif
}

void Application::Kill() {
#ifdef DEBUG_APP
  printf("Rank %d: Application::Kill: start\n", Application::GetApplication()->GetRank());
#endif
  // GDA pthread_mutex_lock(&lock);
#ifdef DEBUG_APP_LOCK
  printf("Rank %d: Application::Kill lock\n", Application::GetApplication()->GetRank());
#endif
#ifdef DEBUG_APP
  printf("Rank %d: Application::Kill: lock acquired.\n",
         Application::GetApplication()->GetRank())
#endif

  application_done = true;

  theIncomingQueue->Kill();
  theOutgoingQueue->Kill();

#ifdef DEBUG_APP
  printf("Rank %d: Application::Kill: application_done = true. killed in/out queues\n",
         Application::GetApplication()->GetRank());
#endif

  pthread_cond_broadcast(&cond);
  pthread_mutex_unlock(&lock);
#ifdef DEBUG_APP_LOCK
  printf("Rank %d: Application::Kill unlock\n", Application::GetApplication()->GetRank());
#endif

#ifdef DEBUG_APP
  printf("Rank %d: Application::Kill: cond_broadcast(&cond), mutex_unlock(&lock) done\n",
         Application::GetApplication()->GetRank());
#endif

  pthread_join(work_thread_tid, NULL);

#ifdef DEBUG_APP
  printf("Rank %d: Application::Kill: end\n", Application::GetApplication()->GetRank());
#endif
}

void Application::Wait() {
  while (!application_done) {
#ifdef DEBUG_APP_LOCK
  printf("Rank %d: Application::Wait waiting for applciation_done =true. blocked on cond\n",
         Application::GetApplication()->GetRank());
#endif
    pthread_cond_wait(&cond, &lock);
  }
#ifdef DEBUG_APP_LOCK
  printf("Rank %d: Application::Wait cond unblocked\n", Application::GetApplication()->GetRank());
#endif
  pthread_mutex_unlock(&lock);
#ifdef DEBUG_APP_LOCK
  printf("Rank %d: Application::Wait unlock\n", Application::GetApplication()->GetRank());
#endif
  GetMessageManager()->Wait();
}

void *Application::workThread(void *p) {
  Application *theApplication = (Application *)p;

  pthread_mutex_lock(&theApplication->lock);
#ifdef DEBUG_APP_LOCK
  printf("Rank %d: Application::workThread lock\n", Application::GetApplication()->GetRank());
#endif
  pthread_cond_signal(&theApplication->cond);
  pthread_mutex_unlock(&theApplication->lock);
#ifdef DEBUG_APP_LOCK
  printf("Rank %d: Application::workThread unlock\n", Application::GetApplication()->GetRank());
#endif

  Message *m;
#if !DEBUG_DISABLE_WORKTHREAD
  while (theApplication->Running() && (m = theApplication->GetIncomingMessageQueue()->Dequeue()) != NULL) {
    Work *w = theApplication->Deserialize(m);
    delete m;

    if (w->Action()) {
      theApplication->Kill();
    }

    delete w;
  }

  pthread_exit(NULL);
#endif
}
#endif /* GVT_CORE_MPI_APPLICATION_H */
