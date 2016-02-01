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
// TraceDoneWork.cpp
//

#include "gvt/render/unit/TraceDoneWork.h"
#include "gvt/render/unit/MpiRenderer.h"
#include "gvt/core/mpi/Work.h"
#include "gvt/render/actor/Ray.h"

#include <pthread.h>

using namespace std;
using namespace gvt::core::mpi;
using namespace gvt::render::unit;
using namespace gvt::render::actor;

// #define DEBUG_RAY_WORK
#define DEBUG_RAY_TRANSFER

WORK_CLASS(TraceDoneWork)

void TraceDoneWork::Serialize(size_t &size, unsigned char *&serialized) {

  size = 2 * sizeof(int);
  serialized = static_cast<unsigned char *>(malloc(size));

  unsigned char *buf = serialized;

  *reinterpret_cast<int *>(buf) = done;
  buf += sizeof(int);
  *reinterpret_cast<int *>(buf) = senderRank;
}

Work *TraceDoneWork::Deserialize(size_t size, unsigned char *serialized) {

  if (size != 2 * sizeof(int)) {
    std::cerr << "Test deserializer ctor with size != 2 * sizeof(int)\n";
    exit(1);
  }

  unsigned char *buf = serialized;

  TraceDoneWork *work = new TraceDoneWork;

  work->done = *reinterpret_cast<int *>(buf);
  buf += sizeof(int);
  work->senderRank = *reinterpret_cast<int *>(buf);

  return work;
}

bool TraceDoneWork::Action() {

  MpiRenderer *renderer = static_cast<MpiRenderer *>(Application::GetApplication());

  pthread_mutex_lock(&renderer->enableDoneActionLock);
  while (!renderer->enableDoneAction) {
    pthread_cond_wait(&renderer->enableDoneActionCondition, &renderer->enableDoneActionLock);
  }
  pthread_mutex_unlock(&renderer->enableDoneActionLock);

#ifdef DEBUG_RAY_TRANSFER
  printf("Rank %d: TraceDoneWork start\n", renderer->GetRank());
#endif

  pthread_mutex_lock(&renderer->doneLock);

  if (this->done == 1) {
    ++renderer->numDones;
  }

  ++(renderer->numDoneSenders);
  
  int numRanks = renderer->GetSize();
  bool done = (renderer->numDoneSenders == numRanks - 1) && (renderer->numDones == numRanks - 1);

  if (done) {
    renderer->allOthersDone = true;
#ifdef DEBUG_RAY_TRANSFER
    printf("Rank %d: TraceDoneWork: all other processes are done with their work !!!", renderer->GetRank());
#endif
  }

#ifdef DEBUG_RAY_TRANSFER
  printf("Rank %d: TraceDoneWork: numDoneSenders: %d numDones: %d\n", renderer->GetRank(), renderer->numDoneSenders, renderer->numDones);
#endif

  if (renderer->numDoneSenders == numRanks - 1) {

#ifdef DEBUG_RAY_TRANSFER
    printf("Rank %d: TraceDoneWork: signaling doneCondition!!!\n", renderer->GetRank());
#endif

    pthread_mutex_lock(&renderer->enableDoneActionLock);
    renderer->enableDoneAction = false;
    pthread_mutex_unlock(&renderer->enableDoneActionLock);

    pthread_cond_signal(&renderer->doneCondition);
  }
  pthread_mutex_unlock(&renderer->doneLock);

  return false;
}
