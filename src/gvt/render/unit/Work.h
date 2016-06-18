
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

#ifndef GVT_RENDER_UNIT_WORK_H
#define GVT_RENDER_UNIT_WORK_H

#include "gvt/render/unit/Contents.h"
#include "gvt/render/unit/Communicator.h"

#include <cassert>
#include <memory>
#include <mpi.h>

#define REGISTER_WORK(ClassName)                                 \
 public:                                                         \
  static void Register(Communicator* comm) {                     \
    ClassName::tag = comm->RegisterWork(ClassName::Deserialize); \
  }                                                              \
  static Work* Deserialize() { return new ClassName; }           \
                                                                 \
  virtual int GetTag() const { return ClassName::tag; }          \
                                                                 \
 private:                                                        \
  static int tag;

#define STATIC_WORK_TAG(ClassName) int ClassName::tag = -1;

namespace gvt {
namespace render {
namespace unit {

class Worker;

class Work {
 public:
  enum CommunicationType { P2P = 0, SEND_ALL, SEND_ALL_OTHER };
  Work() {}

  virtual ~Work() {}

  virtual int GetTag() const {
    assert(false);
    return -1;
  }

  // return type: delete this (1), don't delete this (0)
  virtual bool Action(Worker* worker) {
    assert(false);
    return true;
  }

  std::shared_ptr<Contents> GetContents() { return contents; }

  void Clone(Work* work) { contents = work->GetContents(); }

  int GetSize() const {
    if (!contents) return 0;
    return contents->GetSize();
  }

  void Send(int dest, Communicator* comm) {
    comm->Send(dest, this);
  }

  void SendAll(Communicator* comm) {
    comm->SendAll(this);
  }

  void SendAllOther(Communicator* comm) {
    comm->SendAllOther(this);
  }

 protected:
  friend class Communicator;

  MPI_Request& GetMpiRequest() { return mpiRequest; }

  bool IsSent() {
    int flag;
    MPI_Test(&mpiRequest, &flag, &mpiStatus);
    return flag;
  }

  template <class T>
  const T* GetBufferPtr() const {
    assert(contents);
    return reinterpret_cast<const T*>(contents->Get());
  }

  template <class T>
  T* GetBufferPtr() {
    assert(contents);
    return reinterpret_cast<T*>(contents->Get());
  }

  void Allocate(std::size_t n) {
    assert(!contents);
    assert(n > 0);
    contents.reset(new Contents(n));
  }

  std::shared_ptr<Contents> contents;

  MPI_Request mpiRequest; // for mpi_isend
  MPI_Status mpiStatus;
};

}  // namespace unit
}  // namespace render
}  // namespace gvt

#endif
