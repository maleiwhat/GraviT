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

// #define DEBUG_DONE_TEST_WORK

using namespace gvt::render::unit;

WORK_CLASS(DoneTestWork)

void DoneTestWork::Serialize(size_t &size, unsigned char *&serialized) {
  size = 0;
  serialized = NULL;
  ;
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

bool DoneTestWork::Action() {

  renderer = static_cast<MpiRenderer *>(Application::GetApplication());
  int numRanks = renderer->GetSize();
  std::vector<int> buf(numRanks, 0);

  // TODO (hpark): find the way to get rid of (or improve) this synchronization
  // check if all ranks are ready for the done test
  bool ready = false;
  do {
    int running = renderer->isDoneTestRunning();
    MPI_Allgather(&running, 1, MPI_INT, &buf[0], 1, MPI_INT, MPI_COMM_WORLD);
    ready = allFlagsSet(buf);
  } while (!ready);

  buf = std::vector<int>(numRanks, 0);

  // do the done test
  int empty = renderer->isRayQueueEmpty();
  MPI_Allgather(&empty, 1, MPI_INT, &buf[0], 1, MPI_INT, MPI_COMM_WORLD);

  if (allFlagsSet(buf)) renderer->setAllWorkDone();

  renderer->clearDoneTestRunning();

  return false;
}