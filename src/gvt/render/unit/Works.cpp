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
// Works.cpp
//

#include "gvt/render/unit/Works.h"
#include "gvt/render/unit/MpiRenderer.h"
#include "gvt/core/mpi/Work.h"
#include "gvt/render/actor/Ray.h"

#include <pthread.h>

using namespace std;
using namespace gvt::core::mpi;
using namespace gvt::render::unit;
using namespace gvt::render::actor;

WORK_CLASS(RayTransferWork)

void RayTransferWork::Serialize(size_t &size, unsigned char *&serialized) {
  if (transferType == Request) {
    RayVector& rayV = *outgoingRays;
    // assume raySize > 0
    int raySize = 0;
    for (int i = 0; i < numRays; ++i) {
      raySize += rayV[i].packedSize();
    }
    size = 4 * sizeof(int) + raySize;
    serialized = static_cast<unsigned char *>(malloc(size));
    
    unsigned char *buf = serialized;
    
    *reinterpret_cast<int *>(buf) = transferType;
    buf += sizeof(int);
    *reinterpret_cast<int *>(buf) = senderRank;
    buf += sizeof(int);
    *reinterpret_cast<int *>(buf) = instanceId;
    buf += sizeof(int);
    *reinterpret_cast<int *>(buf) = numRays;
    buf += sizeof(int);

    for (size_t i = 0; i < rayV.size(); ++i) {
      buf += rayV[i].pack(buf);
#ifdef DEBUG_RAY_COPY
      MpiRenderer *renderer = static_cast<MpiRenderer *>(Application::GetApplication());
      if (renderer->GetRank() == 0) {
        std::cout<<"Rank "<<renderer->GetRank()<<": RayTransferWork::serialize: size "<<size
                 <<": senderRank "<<senderRank<<": instanceId "<<instanceId<<": numRays "<<numRays
                 <<": outgoingRays["<<i<<"]{"<<rayV[i]<<"} buf "<<(void*)buf<<std::endl;
      }
#endif
    }
  } else { // Grant
    size = 3 * sizeof(int);
    serialized = static_cast<unsigned char *>(malloc(size));
    unsigned char *buf = serialized;
    
    *reinterpret_cast<int *>(buf) = transferType;
    buf += sizeof(int);
    *reinterpret_cast<int *>(buf) = senderRank;
    buf += sizeof(int);
    *reinterpret_cast<int *>(buf) = numRays;
  }
}

Work *RayTransferWork::Deserialize(size_t size, unsigned char *serialized) {

  unsigned char *buf = serialized;
  RayTransferWork *work = new RayTransferWork;
  work->transferType = *reinterpret_cast<int *>(buf);
  buf += sizeof(int);

  if (work->transferType == Request) {
    work->senderRank = *reinterpret_cast<int *>(buf);
    buf += sizeof(int);
    work->instanceId = *reinterpret_cast<int *>(buf);
    buf += sizeof(int);
    int rayCount = *reinterpret_cast<int *>(buf);
    buf += sizeof(int);
    work->numRays = rayCount;
    
    work->incomingRays.resize(rayCount);
    
    for (int i = 0; i < rayCount; ++i) {
      Ray ray(buf);
      work->incomingRays[i] = ray;
      buf += ray.packedSize();
    }
  } else { // Grant
    work->senderRank = *reinterpret_cast<int *>(buf);
    buf += sizeof(int);
    work->numRays = *reinterpret_cast<int *>(buf);
  }
  return work;
}

void RayTransferWork::setup(int transferType, int senderRank, int instanceId, gvt::render::actor::RayVector *outgoingRays) {
  this->transferType = transferType;
  this->senderRank = senderRank;
  this->instanceId = instanceId;
  this->numRays = outgoingRays->size();
  this->outgoingRays = outgoingRays;

#ifdef DEBUG_RAY_TRANSFER
  printf("Rank %d: RayTransferWork::setRays: instanceId %d: numRays %d\n",
         Application::GetApplication()->GetRank(), instanceId, this->numRays);
#endif

#ifdef DEBUG_RAY_COPY
  MpiRenderer *renderer = static_cast<MpiRenderer *>(Application::GetApplication());
  printf("Rank %d: RayTransferWork::setRays: instanceId %d: numRays %d\n", renderer->GetRank(), instanceId, numRays);
  if (renderer->GetRank() == 0) {
    std::cout<<"setRays: rays[0]{"<<(*rays)[0]<<"}"<<std::endl;
    std::cout<<"setRays: rays[1]{"<<(*rays)[1]<<"}"<<std::endl;
    std::cout<<"setRays: outgoingRays[0]{"<<(*rays)[0]<<"}"<<std::endl;
    std::cout<<"setRays: outgoingRays[1]{"<<(*rays)[1]<<"}"<<std::endl;
  }
#endif
}

void RayTransferWork::setup(int transferType, int senderRank, int numRays) {
  this->transferType = transferType;
  this->numRays = numRays;
}

void RayTransferWork::copyIncomingRays(std::map<int, gvt::render::actor::RayVector> *destinationRayQ) {
  // TODO: avoid resizing
  if (destinationRayQ->find(instanceId) != destinationRayQ->end()) {
    (*destinationRayQ)[instanceId].insert((*destinationRayQ)[instanceId].end(), incomingRays.begin(), incomingRays.end()); 
  } else {
    (*destinationRayQ)[instanceId] = incomingRays;
  }
}

bool RayTransferWork::Action() {
  MpiRenderer *renderer = static_cast<MpiRenderer *>(Application::GetApplication());
  if (transferType == Request) {
    renderer->bufferRayTransferWork(this);
    // renderer->copyIncomingRays(instanceId, &incomingRays);
  } else { // Grant
    renderer->applyRayTransferResult(numRays);
  }
  return false;
}

bool RayTransferWork::deferDeletingThis() {
  return (transferType == Request);
};

WORK_CLASS(VoteWork)

void VoteWork::Serialize(size_t &size, unsigned char *&serialized) {
  size = 3 * sizeof(int);
  serialized = static_cast<unsigned char *>(malloc(size));
  unsigned char *buf = serialized;
  
  *reinterpret_cast<int *>(buf) = voteType;
  buf += sizeof(int);
  *reinterpret_cast<int *>(buf) = senderRank;
  buf += sizeof(int);
  *reinterpret_cast<unsigned int *>(buf) = timeStamp;
}

Work *VoteWork::Deserialize(size_t size, unsigned char *serialized) {

  unsigned char *buf = serialized;
  VoteWork *work = new VoteWork;

  work->voteType = *reinterpret_cast<int *>(buf);
  buf += sizeof(int);
  work->senderRank = *reinterpret_cast<int *>(buf);
  buf += sizeof(int);
  work->timeStamp = *reinterpret_cast<unsigned int *>(buf);

  return work;
}

void VoteWork::setup(int voteType, int senderRank, unsigned int timeStamp) {
  this->voteType = voteType;
  this->senderRank = senderRank;
  this->timeStamp = timeStamp;
}

bool VoteWork::Action() {
  MpiRenderer *renderer = static_cast<MpiRenderer *>(Application::GetApplication());
  if (voteType == NoWork) {
    renderer->voteForNoWork(senderRank, timeStamp);
  } else if (voteType == Resign) {
    renderer->voteForResign(senderRank, timeStamp);
  } else { // Commit or Abort
    renderer->applyVoteResult(voteType, timeStamp);
  }
  return false;
}

WORK_CLASS(PixelGatherWork)

void PixelGatherWork::Serialize(size_t &size, unsigned char *&serialized) {
  size = 0;
  serialized = NULL;
}

Work *PixelGatherWork::Deserialize(size_t size, unsigned char *serialized) {
  if (size != 0) {
    std::cerr << "PixelGatherWork deserializer call with size != 0 rank " << Application::GetApplication()->GetRank()
              << "\n";
    exit(1);
  }
  PixelGatherWork *work = new PixelGatherWork;
  return work;
}

bool PixelGatherWork::Action() {
  MpiRenderer *renderer = static_cast<MpiRenderer *>(Application::GetApplication());
  renderer->compositePixels();
  return false;
}

WORK_CLASS(TimeGatherWork)

void TimeGatherWork::Serialize(size_t &size, unsigned char *&serialized) {
  size = 0;
  serialized = NULL;
}

Work *TimeGatherWork::Deserialize(size_t size, unsigned char *serialized) {
  if (size != 0) {
    std::cerr << "TimeGatherWork deserializer call with size != 0 rank " << Application::GetApplication()->GetRank()
              << "\n";
    exit(1);
  }
  TimeGatherWork *work = new TimeGatherWork;
  return work;
}

bool TimeGatherWork::Action() {
  MpiRenderer *renderer = static_cast<MpiRenderer *>(Application::GetApplication());
  renderer->gatherTimes();
  return false;
}

