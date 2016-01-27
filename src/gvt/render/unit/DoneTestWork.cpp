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
// DoneTestWork.cpp
//

#include "gvt/render/unit/DoneTestWork.h"
#include "gvt/render/unit/MpiRenderer.h"

#include <vector>
#include <iostream>
#include <tbb/mutex.h>

// #define DEBUG_DONE_TEST_WORK

using namespace gvt::render::unit;

WORK_CLASS(DoneTestWork)

void DoneTestWork::Serialize(size_t &size, unsigned char *&serialized) {
  size = 0;
  serialized = NULL;
}

Work *DoneTestWork::Deserialize(size_t size, unsigned char *serialized) {
  if (size != 0) {
    std::cerr << "DoneTestWork deserializer call with size != 0 rank " << Application::GetApplication()->GetRank()
              << "\n";
    exit(1);
  }
  DoneTestWork *work = new DoneTestWork;
  return work;
}

bool DoneTestWork::allFlagsSet(std::vector<int> &buf) {
  int count = 0;
  for (int i = 0; i < buf.size(); ++i) count += buf[i];
  return (count == buf.size());
}

// #define FIND_THE_BUG

bool DoneTestWork::Action() {

  renderer = static_cast<MpiRenderer *>(Application::GetApplication());
  int numRanks = renderer->GetSize();

  // TODO (hpark): find the way to get rid of (or improve) this synchronization
  // check if all ranks are ready for the done test
  int readyForTest = 0;
  do {
    int running = renderer->isDoneTestRunning();
    MPI_Allreduce(&running, &readyForTest, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD);
  } while(readyForTest != numRanks);

  {
    tbb::mutex* remoteRayQMutex= renderer->getIncomingRayQueueMutex();
    tbb::mutex::scoped_lock remoteRayQLock(*remoteRayQMutex);
  
    // do the done test
    unsigned int statusTx[4] = {renderer->isRayQueueEmpty(),
                                renderer->isIncomingRayQueueEmpty(),
                                renderer->getNumRaysSent(),
                                renderer->getNumRaysReceived()};
    unsigned int statusRx[4] = {0, 0, 0, 0};
    MPI_Allreduce(statusTx, statusRx, 4, MPI_UNSIGNED, MPI_SUM, MPI_COMM_WORLD);

    bool done = (statusRx[0] == numRanks &&
                 statusRx[1] == numRanks &&
                 statusRx[2] == statusRx[3]);
#ifdef FIND_THE_BUG
    printf("DoneTestWork::Action: Rank %d: MQ %d IQ %d Sent %d Received %d\n",
            renderer->GetRank(), statusRx[0],
            statusRx[1], statusRx[2], statusRx[3]);
#endif

    if (done) {
      renderer->setAllWorkDone();
    }
    renderer->clearDoneTestRunning();
  }
  return false;
}