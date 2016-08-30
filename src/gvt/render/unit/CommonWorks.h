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
// CommonWorks.h
//

#ifndef GVT_RENDER_UNIT_COMMON_WORKS_H
#define GVT_RENDER_UNIT_COMMON_WORKS_H

#include "gvt/render/unit/Work.h"
#include "gvt/render/unit/Worker.h"

#include <cassert>
#include <memory>

namespace gvt {
namespace render {
namespace unit {

/**
 * This class can be used to send a predefined command to other processes.
 */
class Command : public Work {
  REGISTER_WORK(Command)

public:
  enum Type {
    QUIT = 0
  };

  struct Data {
    int type;
  };

  Command() : Work() { Command(0); }
  Command(int type) : Work() {
    Allocate(sizeof(Data));
    GetBufferPtr<Data>()->type = type;
  }

  virtual bool Action(Worker *worker);

  // getters
  int GetType() const { return GetBufferPtr<Data>()->type; }
};

/**
 * This class is for test purposes only.
 */
class PingTest : public Work {
  REGISTER_WORK(PingTest)

public:
  struct Data {
    int value;
  };

  PingTest() : Work() { PingTest(0); }
  PingTest(int value) : Work() {
    Allocate(sizeof(Data));
    GetBufferPtr<Data>()->value = value;
  }

  virtual bool Action(Worker *worker);

  // getters
  int GetValue() const { return GetBufferPtr<Data>()->value; }
};

/**
 * This class is for compositing frame buffers (temporarily not used).
 */
class Composite : public Work {
  REGISTER_WORK(Composite)

  struct Data {
    int value;
  };

public:
  Composite() : Work() { Composite(0); }
  Composite(int value) : Work() {
    Allocate(sizeof(Data));
    GetBufferPtr<Data>()->value = 0;
  }

  virtual bool Action(Worker *worker);

  int GetValue() const { return GetBufferPtr<Data>()->value; }
};

} // namespace unit
} // namespace render
} // namespace gvt

#endif
