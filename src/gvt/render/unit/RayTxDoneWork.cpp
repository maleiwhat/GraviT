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
// RayTxDoneWork.cpp
//

#include "gvt/render/unit/RayTxDoneWork.h"
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

WORK_CLASS(RayTxDoneWork)

void RayTxDoneWork::Serialize(size_t &size, unsigned char *&serialized) {

  size = 0;
  serialized = NULL;
}

Work *RayTxDoneWork::Deserialize(size_t size, unsigned char *serialized) {

  if (size != 0) {
    std::cerr << "AllG deserializer call with size != 0 rank " << Application::GetApplication()->GetRank() << "\n";
    exit(1);
  }

  RayTxDoneWork *work = new RayTxDoneWork;
  return work;
}

bool RayTxDoneWork::Action() {

  MpiRenderer *renderer = static_cast<MpiRenderer *>(Application::GetApplication());

  bool allSenders = false;

  pthread_mutex_lock(&renderer->rayTxDoneLock);

  ++(renderer->numRayTxDoneSenders);
  allSenders = (renderer->numRayTxDoneSenders == renderer->GetSize() - 1);

  if (allSenders) {

    renderer->numRayTxDoneSenders = 0;

    pthread_mutex_lock(&renderer->rayCommitReadyLock);
    renderer->rayCommitReady = true;

#ifdef DEBUG_RAY_TRANSFER
    printf("Rank %d: RayTxDoneWork: signal rayCommitReadyCond\n", renderer->GetRank());
#endif
    pthread_cond_signal(&renderer->rayCommitReadyCond);
    pthread_mutex_unlock(&renderer->rayCommitReadyLock);
  }
  
  pthread_mutex_unlock(&renderer->rayTxDoneLock);

  return false;
}
