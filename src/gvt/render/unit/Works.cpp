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
#include "gvt/core/mpi/Work.h"
#include "gvt/render/actor/Ray.h"
#include "gvt/render/unit/MpiRenderer.h"

#include <pthread.h>

using namespace std;
using namespace gvt::core::mpi;
using namespace gvt::render::unit;
using namespace gvt::render::actor;

WORK_CLASS_TYPE(RayTransferWork)

// void RayTransferWork::Serialize(size_t &size, unsigned char *&serialized) {
//   if (transferType == Request) {
//     RayVector &rayV = *outgoingRays;
//     // assume raySize > 0
//     int raySize = rayV.size() * sizeof(gvt::render::actor::Ray);
//     size = 4 * sizeof(int) + raySize;
//     serialized = static_cast<unsigned char *>(malloc(size));
//
//     unsigned char *buf = serialized;
//
//     *reinterpret_cast<int *>(buf) = transferType;
//     buf += sizeof(int);
//     *reinterpret_cast<int *>(buf) = senderRank;
//     buf += sizeof(int);
//     *reinterpret_cast<int *>(buf) = instanceId;
//     buf += sizeof(int);
//     *reinterpret_cast<int *>(buf) = numRays;
//     buf += sizeof(int);
//
//     std::memcpy(buf, &rayV[0], raySize);
//   } else { // Grant
//     size = 3 * sizeof(int);
//     serialized = static_cast<unsigned char *>(malloc(size));
//     unsigned char *buf = serialized;
//
//     *reinterpret_cast<int *>(buf) = transferType;
//     buf += sizeof(int);
//     *reinterpret_cast<int *>(buf) = senderRank;
//     buf += sizeof(int);
//     *reinterpret_cast<int *>(buf) = numRays;
//   }
// }

// Work *RayTransferWork::Deserialize(size_t size, unsigned char *serialized) {
//
//   unsigned char *buf = serialized;
//   RayTransferWork *work = new RayTransferWork;
//   work->transferType = *reinterpret_cast<int *>(buf);
//   buf += sizeof(int);
//
//   if (work->transferType == Request) {
//     work->senderRank = *reinterpret_cast<int *>(buf);
//     buf += sizeof(int);
//     work->instanceId = *reinterpret_cast<int *>(buf);
//     buf += sizeof(int);
//     int rayCount = *reinterpret_cast<int *>(buf);
//     buf += sizeof(int);
//     work->numRays = rayCount;
//
//     work->incomingRays.reserve(rayCount);
//     std::memmove(&work->incomingRays[0], buf, rayCount *
//     sizeof(gvt::render::actor::Ray));
//     work->incomingRays.resize(rayCount);
//   } else { // Grant
//     work->senderRank = *reinterpret_cast<int *>(buf);
//     buf += sizeof(int);
//     work->numRays = *reinterpret_cast<int *>(buf);
//   }
//   return work;
// }

void RayTransferWork::setup(int transferType, int senderRank, int numRays) {
  // this->transferType = transferType;
  // this->numRays = numRays;
  RayInfo info;
  info.transferType = transferType;
  info.senderRank = senderRank;
  info.instanceId = 0;  // unused
  info.numRays = numRays;
  memcpy(contents->get(), &info, sizeof(RayInfo));
}

void RayTransferWork::setup(int transferType, int senderRank, int instanceId,
                            const gvt::render::actor::RayVector &outgoingRays) {
  // this->transferType = transferType;
  // this->senderRank = senderRank;
  // this->instanceId = instanceId;
  // this->numRays = outgoingRays->size();
  // this->outgoingRays = outgoingRays;
  RayInfo info;
  info.transferType = transferType;
  info.senderRank = senderRank;
  info.instanceId = instanceId;
  info.numRays = outgoingRays.size();

  unsigned char *buf = contents->get();
  memcpy(buf, &info, sizeof(RayInfo));
  memcpy(buf + sizeof(RayInfo), &outgoingRays[0],
         info.numRays * sizeof(gvt::render::actor::Ray));

#ifndef NDEBUG
  printf("rank %d tx_type %d sender %d instance %d num_rays %d in %s\n",
         Application::GetTheApplication()->GetRank(), info.transferType,
         info.senderRank, info.instanceId, info.numRays, __PRETTY_FUNCTION__);

  MpiRenderer *renderer =
      static_cast<MpiRenderer *>(Application::GetTheApplication());
  std::cout << "setRays: outgoingRays[0]{" << outgoingRays[0] << "}"
            << std::endl;
#endif
}

void RayTransferWork::copyIncomingRays(
    std::map<int, gvt::render::actor::RayVector> *destinationRayQ) {
  RayInfo info = getRayInfo();
  int instanceId = info.instanceId;
  std::size_t srcSize = info.numRays * sizeof(gvt::render::actor::Ray);
  if (destinationRayQ->find(instanceId) != destinationRayQ->end()) {
    (*destinationRayQ)[instanceId].insert((*destinationRayQ)[instanceId].end(),
                                          incomingRays.begin(),
                                          incomingRays.end());
    // gvt::render::actor::RayVector& rays = (*destinationRayQ)[instanceId];
    // std::size_t destSize = rays.size();
    // rays.resize(srcSize + destSize); // TODO: avoid this
    // memcpy(&rays[destSize], getRays(), srcSize);
  } else {
    (*destinationRayQ)[instanceId] = incomingRays;
    // (*destinationRayQ)[instanceId] = gvt::render::actor::RayVector();
    // gvt::render::actor::RayVector& rays = (*destinationRayQ)[instanceId];
    // rays.resize(srcSize); // TODO: avoid this
    // memcpy(&rays[0], getRays(), srcSize);
  }
  // // TODO: avoid resizing
  // if (destinationRayQ->find(instanceId) != destinationRayQ->end()) {
  //   (*destinationRayQ)[instanceId]
  //       .insert((*destinationRayQ)[instanceId].end(), incomingRays.begin(),
  //       incomingRays.end());
  // } else {
  //   (*destinationRayQ)[instanceId] = incomingRays;
  // }
}

bool RayTransferWork::Action() {
  MpiRenderer *renderer =
      static_cast<MpiRenderer *>(Application::GetTheApplication());
  RayInfo info = getRayInfo();
#ifndef NDEBUG
  printf("ranks %d RayInfo (type %d sender %d instance %d num_rays %d) in %s\n",
         renderer->GetRank(), info.transferType, info.senderRank,
         info.instanceId, info.numRays, __PRETTY_FUNCTION__);
#endif
  if (info.transferType == Request) {
    incomingRays.resize(info.numRays);
    unsigned char *buf = contents->get();
    memcpy(&incomingRays[0], buf + sizeof(RayInfo),
           info.numRays * sizeof(gvt::render::actor::Ray));
    renderer->bufferRayTransferWork(this);
    // renderer->copyIncomingRays(instanceId, &incomingRays);
  } else if (info.transferType == Grant) {  // Grant
    renderer->applyRayTransferResult(info.numRays);
  } else {
    assert(false);
  }
  return false;
}

bool RayTransferWork::deferDeletingThis() {
  RayInfo info = getRayInfo();
  return (info.transferType == Request);
};

WORK_CLASS_TYPE(VoteWork)

// void VoteWork::Serialize(size_t &size, unsigned char *&serialized) {
//   size = 3 * sizeof(int);
//   serialized = static_cast<unsigned char *>(malloc(size));
//   unsigned char *buf = serialized;
//
//   *reinterpret_cast<int *>(buf) = voteType;
//   buf += sizeof(int);
//   *reinterpret_cast<int *>(buf) = senderRank;
// }

// Work *VoteWork::Deserialize(size_t size, unsigned char *serialized) {
//
//   unsigned char *buf = serialized;
//   VoteWork *work = new VoteWork;
//
//   work->voteType = *reinterpret_cast<int *>(buf);
//   buf += sizeof(int);
//   work->senderRank = *reinterpret_cast<int *>(buf);
//
//   return work;
// }

void VoteWork::setup(int voteType, int senderRank) {
  // this->voteType = voteType;
  // this->senderRank = senderRank;
  Info info;
  info.voteType = voteType;
  info.senderRank = senderRank;
  memcpy(contents->get(), &info, sizeof(Info));
}

bool VoteWork::Action() {
  MpiRenderer *renderer =
      static_cast<MpiRenderer *>(Application::GetTheApplication());
  Info info = getInfo();
  if (info.voteType == PROPOSE) {
    renderer->setProposeAvailable();
  } else if (info.voteType == DO_COMMIT) {
    renderer->commit();
  } else if (info.voteType == DO_ABORT) {
    renderer->abort();
  } else if (info.voteType == VOTE_COMMIT) {
    renderer->voteCommit();
  } else if (info.voteType == VOTE_ABORT) {
    renderer->voteAbort();
  } else {
    printf("rank %d: invalid vote type %d\n", renderer->GetRank(),
           info.voteType);
    exit(1);
  }
  return false;
}

WORK_CLASS_TYPE(PixelGatherWork)

// void PixelGatherWork::Serialize(size_t &size, unsigned char *&serialized) {
//   size = 0;
//   serialized = NULL;
// }
//
// Work *PixelGatherWork::Deserialize(size_t size, unsigned char *serialized) {
//   if (size != 0) {
//     std::cerr << "PixelGatherWork deserializer call with size != 0 rank " <<
//     Application::GetTheApplication()->GetRank()
//               << "\n";
//     exit(1);
//   }
//   PixelGatherWork *work = new PixelGatherWork;
//   return work;
// }

bool PixelGatherWork::Action(MPI_Comm comm) {
#ifndef NDEBUG
  printf("start of %s\n", __PRETTY_FUNCTION__);
#endif
  MpiRenderer *renderer =
      static_cast<MpiRenderer *>(Application::GetTheApplication());
  renderer->gatherFramebuffers();
  return false;
}

WORK_CLASS_TYPE(TimeGatherWork)

// void TimeGatherWork::Serialize(size_t &size, unsigned char *&serialized) {
//   size = 0;
//   serialized = NULL;
// }
//
// Work *TimeGatherWork::Deserialize(size_t size, unsigned char *serialized) {
//   if (size != 0) {
//     std::cerr << "TimeGatherWork deserializer call with size != 0 rank " <<
//     Application::GetTheApplication()->GetRank()
//               << "\n";
//     exit(1);
//   }
//   TimeGatherWork *work = new TimeGatherWork;
//   return work;
// }

bool TimeGatherWork::Action(MPI_Comm comm) {
  MpiRenderer *renderer =
      static_cast<MpiRenderer *>(Application::GetTheApplication());
  renderer->gatherTimes();
  return false;
}

