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
// RayCommitDoneWork.cpp
//

#include "gvt/render/unit/RayCommitDoneWork.h"
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

WORK_CLASS(RayCommitDoneWork)

void RayCommitDoneWork::Serialize(size_t &size, unsigned char *&serialized) {

  size = sizeof(int);
  serialized = static_cast<unsigned char *>(malloc(size));

  *reinterpret_cast<int *>(serialized) = isRayQueueEmpty;

#ifdef DEBUG_RAY_TRANSFER
  printf("Rank %d: RayCommitDoneWork::Serialize: size %lu: isRayQueueEmpty %d\n",
         Application::GetApplication()->GetRank(), size, isRayQueueEmpty);
#endif
}

Work *RayCommitDoneWork::Deserialize(size_t size, unsigned char *serialized) {

  if (size != sizeof(int)) {
    std::cerr << "AllG deserializer call with size != sizeof(int) rank "
              << Application::GetApplication()->GetRank() << "\n";
    exit(1);
  }

  RayCommitDoneWork *work = new RayCommitDoneWork;
  work->isRayQueueEmpty = *reinterpret_cast<int *>(serialized);

#ifdef DEBUG_RAY_TRANSFER
  printf("Rank %d: RayCommitDoneWork::Deserialize: size %lu: isRayQueueEmpty %d\n",
         Application::GetApplication()->GetRank(), size, work->isRayQueueEmpty);
#endif

  return work;
}

bool RayCommitDoneWork::Action() {

  MpiRenderer *renderer = static_cast<MpiRenderer *>(Application::GetApplication());

  pthread_mutex_lock(&renderer->rayCommitDoneLock);

  ++(renderer->numRayCommitDoneSenders);
  bool allSenders = (renderer->numRayCommitDoneSenders == renderer->GetSize() - 1);

  if (isRayQueueEmpty == 1)
    ++(renderer->numRayQEmptyFlags);

#ifdef DEBUG_RAY_TRANSFER
    printf("Rank %d: RayCommitDoneWork: numRayCommitDoneSenders %d: numRayQEmptyFlags %d\n",
           renderer->GetRank(), renderer->numRayCommitDoneSenders, renderer->numRayQEmptyFlags);
#endif

  if (allSenders) {

    if (renderer->numRayQEmptyFlags == renderer->GetSize() - 1) {
      renderer->allOtherProcessesDone = true;
    }

    renderer->numRayCommitDoneSenders = 0;
    renderer->numRayQEmptyFlags = 0;

    pthread_mutex_lock(&renderer->workRestartReadyLock);
    renderer->workRestartReady = true;

#ifdef DEBUG_RAY_TRANSFER
    printf("Rank %d: RayCommitDoneWork: signal workRestartReadyCond: allOtherProcessesDone %d\n",
           renderer->GetRank(), renderer->allOtherProcessesDone);
#endif
    pthread_cond_signal(&renderer->workRestartReadyCond);
    pthread_mutex_unlock(&renderer->workRestartReadyLock);
  }

  pthread_mutex_unlock(&renderer->rayCommitDoneLock);
  

  return false;
}
