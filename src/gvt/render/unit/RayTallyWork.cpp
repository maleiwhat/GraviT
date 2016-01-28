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
// RayTallyWork.cpp
//

#include "gvt/render/unit/RayTallyWork.h"
#include "gvt/render/unit/MpiRenderer.h"
#include "gvt/core/mpi/Work.h"
#include "gvt/render/actor/Ray.h"

using namespace std;
using namespace gvt::core::mpi;
using namespace gvt::render::unit;
using namespace gvt::render::actor;

// #define DEBUG_RAY_WORK
#define FIND_THE_BUG

WORK_CLASS(RayTallyWork)

void RayTallyWork::Serialize(size_t &size, unsigned char *&serialized) {
  size = 0;
  serialized = NULL;
}

Work *RayTallyWork::Deserialize(size_t size, unsigned char *serialized) {
  if (size != 0) {
    std::cerr << "DoneTestWork deserializer call with size != 0 rank " << Application::GetApplication()->GetRank()
              << "\n";
    exit(1);
  }
  RayTallyWork *work = new RayTallyWork;
  return work;
}

bool RayTallyWork::Action() {

  MpiRenderer *renderer = static_cast<MpiRenderer *>(Application::GetApplication());
  std::map<int, RayVector> *rayQ = renderer->getRayQueue();
  int numRanks = renderer->GetSize();
  int myRank = renderer->GetRank();

  // count the number of rays to send
  std::vector<unsigned int> rayCounts = std::vector<unsigned int>(numRanks, 0);

  for (auto &q : *rayQ) {
    int instance = q.first;
    RayVector& rays = q.second; 
    int ownerRank = renderer->getInstanceOwner(instance);
    if (ownerRank != myRank) {
      rayCounts[ownerRank] += rays.size();
    }
  }

  unsigned int numRaysToReceive = 0;
  MPI_Allreduce(&rayCounts[myRank], &numRaysToReceive, 1, MPI_UNSIGNED, MPI_SUM, MPI_COMM_WORLD);

  renderer->setNumRaysToReceive(numRaysToReceive);
  
  return false;
}
