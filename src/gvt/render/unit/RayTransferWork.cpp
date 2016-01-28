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
// RayTransferWork.cpp
//

#include "gvt/render/unit/RayTransferWork.h"
#include "gvt/render/unit/MpiRenderer.h"
#include "gvt/core/mpi/Work.h"
#include "gvt/render/actor/Ray.h"

using namespace std;
using namespace gvt::core::mpi;
using namespace gvt::render::unit;
using namespace gvt::render::actor;

// #define DEBUG_RAY_WORK
#define FIND_THE_BUG

WORK_CLASS(RayTransferWork)

void RayTransferWork::Serialize(size_t &size, unsigned char *&serialized) {
  // assume raySize > 0
  size_t raySize = static_cast<size_t>(numRays * outgoingRays[0].packedSize());
  size = 2 * sizeof(int) + raySize;
  serialized = static_cast<unsigned char *>(malloc(size));

  unsigned char *buf = serialized;

  *reinterpret_cast<int *>(buf) = instanceId;
  buf += sizeof(int);
  *reinterpret_cast<int *>(buf) = numRays;
  buf += sizeof(int);

  for (size_t i = 0; i < outgoingRays.size(); ++i) {
    outgoingRays[i].serialize(buf);
  }
}

Work *RayTransferWork::Deserialize(size_t size, unsigned char *serialized) {
  unsigned char *buf = serialized;

  RayTransferWork *work = new RayTransferWork;

  work->instanceId = *reinterpret_cast<int *>(buf);
  buf += sizeof(int);
  int numRays = *reinterpret_cast<int *>(buf);
  buf += sizeof(int);
  work->numRays = numRays;

  // TODO (hpark): need some static function in Ray.h
  //               to evaluate the ray size
  // if not available, do some hand calculation and hard code it here
  Ray dummyRay;
  size_t raysize = dummyRay.packedSize() * numRays;
  if (size != (2 * sizeof(int) + (raysize))) {
    std::cerr << "Test deserializer ctor with received size (" << size << ") != expected size (" << raysize << ")\n";
    exit(1);
  }

  work->incomingRays.resize(numRays);

  for(size_t i = 0; i < numRays; ++i) {
    work->incomingRays[i] = Ray(buf);
  }

  return work;
}

void RayTransferWork::setRays(unsigned int instanceId, gvt::render::actor::RayVector &rays) {
  this->instanceId = instanceId;
  this->numRays = rays.size();
  this->outgoingRays = rays;
}

bool RayTransferWork::Action() {

  MpiRenderer *renderer = static_cast<MpiRenderer *>(Application::GetApplication());
  std::map<int, RayVector> *inRayQ = renderer->getIncomingRayQueue(); 

  // TODO (hpark): for now this lock is unnecessary but for multiple threads in use, we need this in this future.
  // tbb::mutex *remoteRayQMutex = renderer->getIncomingRayQueueMutex(); 
  // {
  //   tbb::mutex::scoped_lock remoteRayQLock(*remoteRayQMutex);

  if (inRayQ->find(instanceId) != inRayQ->end()) {
    (*inRayQ)[instanceId].insert((*inRayQ)[instanceId].end(), incomingRays.begin(), incomingRays.end()); 
  } else {
    (*inRayQ)[instanceId] = incomingRays;
  }

  int numRaysReceived = renderer->incrementNumRaysReceived(numRays);
  if (numRaysReceived == renderer->getNumRaysToReceive()) {
    renderer->setRayTransferDone(true);
  }

#ifdef FIND_THE_BUG
  int myRank = renderer->GetRank();
  printf("RayTransferWork::Action: Rank %d: domain %d: %d\n", myRank, instanceId, numRays);
#endif

  return false;
}
