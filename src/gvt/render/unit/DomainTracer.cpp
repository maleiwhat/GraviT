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

#include "gvt/render/unit/DomainTracer.h"

#include <iostream>

#include "gvt/render/unit/Worker.h"
#include "gvt/render/unit/CommonWorks.h"

namespace gvt {
namespace render {
namespace unit {

#define PING_TEST

void DomainTracer::Trace(Worker* worker) {
#ifndef NDEBUG
  std::cout << __PRETTY_FUNCTION__ << "\n";
#endif
#ifdef PING_TEST
  Work* work = NULL;
  if (worker->GetMpiSize() > 1) {
    int rank = worker->GetRank();
    if (rank == 0) {
      work = new PingTest(rank);
      work->Send(1, worker);
    }
  } else {
    std::cout << "need more than 1 rank to do ping test. Terminating.\n";
    work = new Command(Command::QUIT);
    work->SendAll(worker);
  }
#endif
}

}  // namespace unit
}  // namespace render
}  // namespace gvt

