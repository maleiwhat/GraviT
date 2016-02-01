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
// RayTxWork.cpp
//

#include "gvt/render/unit/RayTxWork.h"
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
#define DEBUG_RAY_COPY

WORK_CLASS(RayTxWork)

void RayTxWork::Serialize(size_t &size, unsigned char *&serialized) {
  // assume raySize > 0
  int raySize = 0;
  for (int i = 0; i < numRays; ++i) {
    raySize += rayBuffer[i].packedSize();
  }
  size = 2 * sizeof(int) + raySize;
  serialized = static_cast<unsigned char *>(malloc(size));

  unsigned char *buf = serialized;

  *reinterpret_cast<int *>(buf) = instanceId;
  buf += sizeof(int);
  *reinterpret_cast<int *>(buf) = numRays;
  buf += sizeof(int);

  for (size_t i = 0; i < rayBuffer.size(); ++i) {
    // rayBuffer[i].serialize(buf);
    buf += rayBuffer[i].pack(buf);
#ifdef DEBUG_RAY_COPY
  MpiRenderer *renderer = static_cast<MpiRenderer *>(Application::GetApplication());
  if (renderer->GetRank() == 0) {
    std::cout<<"Rank "<<renderer->GetRank()<<": RayTxWork::serialize: size "<<size
             <<": instanceId "<<instanceId<<": numRays "<<numRays
             <<": outgoingRays["<<i<<"]{"<<rayBuffer[i]<<"} buf "<<(void*)buf<<std::endl;
  }
#endif
  }
}

Work *RayTxWork::Deserialize(size_t size, unsigned char *serialized) {

  unsigned char *buf = serialized;

  RayTxWork *work = new RayTxWork;

  work->instanceId = *reinterpret_cast<int *>(buf);
  buf += sizeof(int);
  int numRays = *reinterpret_cast<int *>(buf);
  buf += sizeof(int);
  work->numRays = numRays;

  work->rayBuffer.resize(numRays);

  for(size_t i = 0; i < numRays; ++i) {
    Ray ray(buf);
    work->rayBuffer[i] = ray;
    buf += ray.packedSize();
#ifdef DEBUG_RAY_COPY
    MpiRenderer *renderer = static_cast<MpiRenderer *>(Application::GetApplication());
    if (renderer->GetRank() == 1) {
      std::cout<<"Rank "<<renderer->GetRank()<<": RayTxWork::deserialize: size "
               <<size<<": instanceId "<<work->instanceId<<": numRays "<<numRays
               <<": rayBuffer["<<i<<"]{"<<work->rayBuffer[i]<<"} buf "<<(void*)buf<<std::endl;
    }
#endif

  }

  return work;
}

void RayTxWork::setRays(int instanceId, gvt::render::actor::RayVector &rays) {
  this->instanceId = instanceId;
  this->numRays = rays.size();
  this->rayBuffer = rays;

#ifdef DEBUG_RAY_TRANSFER
  printf("Rank %d: RayTxWork::setRays: instanceId %d: numRays %d\n",
         Application::GetApplication()->GetRank(), instanceId, this->numRays);
#endif

#ifdef DEBUG_RAY_COPY
  MpiRenderer *renderer = static_cast<MpiRenderer *>(Application::GetApplication());
  printf("Rank %d: RayTxWork::setRays: instanceId %d: numRays %d\n", renderer->GetRank(), instanceId, numRays);
  if (renderer->GetRank() == 0) {
    std::cout<<"setRays: rays[0]{"<<rays[0]<<"}"<<std::endl;
    std::cout<<"setRays: rays[1]{"<<rays[1]<<"}"<<std::endl;
    std::cout<<"setRays: outgoingRays[0]{"<<rayBuffer[0]<<"}"<<std::endl;
    std::cout<<"setRays: outgoingRays[1]{"<<rayBuffer[1]<<"}"<<std::endl;
  }
#endif
  // rayBuffer.resize(rays.size());
  // for(int i = 0; i < rays.size(); ++i) {
  //   rayBuffer[i] = rays[i];
  // }
}

bool RayTxWork::Action() {

  MpiRenderer *renderer = static_cast<MpiRenderer *>(Application::GetApplication());

// #ifdef DEBUG_RAY_TRANSFER
//   printf("Rank %d: RayTxWork::Action about to start (blocked on enableTransferActionCondition)\n", renderer->GetRank());
// #endif

//   pthread_mutex_lock(&renderer->enableTransferActionLock);
//   while (!renderer->enableTransferAction) {
//     pthread_cond_wait(&renderer->enableTransferActionCondition, &renderer->enableTransferActionLock);
//   }
//   pthread_mutex_unlock(&renderer->enableTransferActionLock);

  pthread_mutex_lock(&renderer->rayBufferLock);

  std::map<int, RayVector> *destination = &renderer->rayBuffer;

// #ifdef DEBUG_RAY_TRANSFER
//   printf("Rank %d: RayTxWork::Action start (enableTransferActionCondition unblocked)\n", renderer->GetRank());
// #endif

  if (destination->find(instanceId) != destination->end()) {
    (*destination)[instanceId].insert((*destination)[instanceId].end(), rayBuffer.begin(), rayBuffer.end()); 
  } else {
    (*destination)[instanceId] = rayBuffer;
  }
 
  // renderer->numRaysReceived += this->numRays;

// #ifdef DEBUG_RAY_TRANSFER
//   printf("Rank %d: RayTxWork: numRaysReceived %d: numRays %d: numRaysToReceive: %d\n",
//          renderer->GetRank(), renderer->numRaysReceived, this->numRays, renderer->numRaysToReceive);
// #endif
#ifdef DEBUG_RAY_TRANSFER
  printf("Rank %d: RayTxWork::Action: instance %d numRays %d\n", renderer->GetRank(), instanceId, numRays);
#endif

  // if (renderer->numRaysReceived > renderer->numRaysToReceive) {
  //   printf("Error: Rank %d: # rays received %d exceeds expected # rays %d\n", renderer->GetRank(), renderer->numRaysReceived, renderer->numRaysToReceive);
  //   exit(1);
  // }

//   if (renderer->numRaysReceived == renderer->numRaysToReceive) {

// #ifdef DEBUG_RAY_TRANSFER
//     printf("Rank %d: RayTxWork: signaling transferCondition!!!\n", renderer->GetRank());
// #endif
    
//     renderer->rayTransferDone = true;

//     pthread_mutex_lock(&renderer->enableTransferActionLock);
//     renderer->enableTransferAction = false;
//     pthread_mutex_unlock(&renderer->enableTransferActionLock);

//     pthread_cond_signal(&renderer->transferCondition);
//   }

  pthread_mutex_unlock(&renderer->rayBufferLock);

  return false;
}
