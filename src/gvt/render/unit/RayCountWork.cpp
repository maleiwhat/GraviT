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
// RayCountWork.cpp
//

#include "gvt/render/unit/RayCountWork.h"
#include "gvt/render/unit/MpiRenderer.h"
#include "gvt/core/mpi/Work.h"
#include "gvt/render/actor/Ray.h"

#include <pthread.h>

using namespace std;
using namespace gvt::core::mpi;
using namespace gvt::render::unit;
using namespace gvt::render::actor;

// #define DEBUG_RAY_WORK
// #define DEBUG_RAY_TRANSFER

WORK_CLASS(RayCountWork)

void RayCountWork::Serialize(size_t &size, unsigned char *&serialized) {

  size = sizeof(int);
  serialized = static_cast<unsigned char *>(malloc(size));
  unsigned char *buf = serialized;
  *reinterpret_cast<int *>(buf) = numRays;
}

Work *RayCountWork::Deserialize(size_t size, unsigned char *serialized) {

  if (size != sizeof(int)) {
    std::cerr << "Test deserializer ctor with size != sizeof(int)\n";
    exit(1);
  }

  unsigned char *buf = serialized;
  RayCountWork *work = new RayCountWork;
  work->numRays = *reinterpret_cast<int *>(buf);

  return work;
}

bool RayCountWork::Action() {

  MpiRenderer *renderer = static_cast<MpiRenderer *>(Application::GetApplication());

#ifdef DEBUG_RAY_TRANSFER
  printf("Rank %d: RayCountWork: blocked on enableTallyActionCondition\n", renderer->GetRank());
#endif

  pthread_mutex_lock(&renderer->enableTallyActionLock);
  while (!renderer->enableTallyAction) {
    pthread_cond_wait(&renderer->enableTallyActionCondition, &renderer->enableTallyActionLock);
  }
  pthread_mutex_unlock(&renderer->enableTallyActionLock);

#ifdef DEBUG_RAY_TRANSFER
  printf("Rank %d: RayCountWork enableTallyActionCondition unblocked\n", renderer->GetRank());
#endif

  pthread_mutex_lock(&renderer->tallyLock);

  ++(renderer->numTallySenders);
  renderer->numRaysToReceive += this->numRays;

  bool done = (renderer->numTallySenders == renderer->GetSize() - 1);

#ifdef DEBUG_RAY_TRANSFER
  printf("Rank %d: RayCountWork: numTallySenders %d: numRays %d: numRaysToReceive %d\n", renderer->GetRank(), renderer->numTallySenders, this->numRays, renderer->numRaysToReceive);
#endif

  if (done) {

#ifdef DEBUG_RAY_TRANSFER
    printf("Rank %d: RayCountWork: signaling tallyCondition!!!\n", renderer->GetRank());
#endif
    renderer->rayTallyDone = true;

    pthread_mutex_lock(&renderer->enableTallyActionLock);
    renderer->enableTallyAction = false;
    pthread_mutex_unlock(&renderer->enableTallyActionLock);

    pthread_cond_signal(&renderer->tallyCondition);

    if (renderer->numRaysToReceive == 0) {
      pthread_mutex_lock(&renderer->transferLock);
      renderer->rayTransferDone = true;
      pthread_cond_signal(&renderer->transferCondition);
      pthread_mutex_unlock(&renderer->transferLock);
    }
  }

  pthread_mutex_unlock(&renderer->tallyLock);

  return false;
}
