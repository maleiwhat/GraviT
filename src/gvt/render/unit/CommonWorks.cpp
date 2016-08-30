/* =======================================================================================
   This file is released as part of GraviT - scalable, platform independent ray tracing
   tacc.github.io/GraviT

   Copyright 2013-2015 Texas Advanced Computing Center, The University of Texas at Austin
   All rights reserved.

   Licensed under the BSD 3-Clause License, (the "License"); you may not use this file
   except in compliance with the License.
   A copy of the License is included with this software in the file LICENSE.
   If your copy does not contain the License, you may obtain a copy of the License at:

       http://opensource.org/licenses/BSD-3-Clause

   Unless required by applicable law or agreed to in writing, software distributed under
   the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
   KIND, either express or implied.
   See the License for the specific language governing permissions and limitations under
   limitations under the License.

   GraviT is funded in part by the US National Science Foundation under awards ACI-1339863,
   ACI-1339881 and ACI-1339840
   ======================================================================================= */
//
// CommonWorks.cpp
//
#include "gvt/render/unit/CommonWorks.h"
#include "gvt/render/unit/Worker.h"
#include <iostream>

namespace gvt {
namespace render {
namespace unit {

STATIC_WORK_TAG(Command)
STATIC_WORK_TAG(Composite)
STATIC_WORK_TAG(PingTest)

bool Command::Action(Worker *worker) {
  int type = GetType();
  if (type == QUIT) {
#ifdef DEBUG_WORK
    std::cout << "rank " << worker->GetRank() << " quitting communicator in " << __PRETTY_FUNCTION__ << std::endl;
#endif
    worker->QuitCommunicator();
  }

  return true;
}

bool Composite::Action(Worker *worker) {
#ifdef DEBUG_WORK
  std::cout << "rank " << worker->GetRank() << " " << __PRETTY_FUNCTION__ << std::endl;
#endif
  RayTracer *tracer = worker->GetTracer();
  tracer->GetProfiler()->Start(profiler::Profiler::COMPOSITE);
  tracer->CompositeFrameBuffers();
  tracer->SignalCompositeDone();
  tracer->GetProfiler()->Stop(profiler::Profiler::COMPOSITE);
  return true;
}

bool PingTest::Action(Worker *worker) {
  Work *work = NULL;
  int rank = worker->GetRank();
  Communicator *comm = worker->GetCommunicator();

  std::cout << "rank " << rank << " got PingTest" << std::endl;

  if (rank == worker->GetMpiSize() - 1) {
    std::cout << "rank " << rank << " broadcasting quit message" << std::endl;
    work = new Command(Command::QUIT);
    work->SendAll(comm);
  } else {
    work = new PingTest(rank);
    work->Send(rank + 1, comm);
  }

  return true;
}

} // namespace unit
} // namespace render
} // namespace gvt

