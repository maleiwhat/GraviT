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
// Work.h
//
#ifndef GVT_RENDER_UNIT_WORK_H
#define GVT_RENDER_UNIT_WORK_H

#include "gvt/render/unit/Contents.h"
#include "gvt/render/unit/Communicator.h"

#include <cassert>
#include <memory>
#include <mpi.h>

/**
 * A macro used to register a new type of work.
 */
#define REGISTER_WORK(ClassName)                                 \
 public:                                                         \
  static void Register(Communicator* comm) {                     \
    ClassName::tag = comm->RegisterWork(ClassName::Deserialize); \
  }                                                              \
  static Work* Deserialize() { return new ClassName; }           \
                                                                 \
  virtual int GetTag() const { return ClassName::tag; }          \
                                                                 \
  static int tag;

/**
 * A MPI tag.
 */
#define STATIC_WORK_TAG(ClassName) int ClassName::tag = -1;

namespace gvt {
namespace render {
namespace unit {

class Worker;

/**
 * The base work class.
 */
class Work {
 public:
  enum CommunicationType {
    P2P = 0, ///< Point-to-point communication.
    SEND_ALL, ///< Send message to all nodes including myself.
    SEND_COLLECTIVE, ///< Same as SEND_ALL.
    SEND_ALL_OTHER ///< Send message to all other nodes excluding myself.
   };

  /**
   * Constructor.
   */
  Work() {}

  /**
   * Deconstructor.
   */
  virtual ~Work() {}

  /**
   * Get the Mpi tag.
   */
  virtual int GetTag() const {
    assert(false);
    return -1;
  }

  /**
   * A function called by the work thread of the receiver's communicator class.
   * \return true: delete this dynamically created work object, false: don't delete this.
   */
  virtual bool Action(Worker* worker) {
    assert(false);
    return true;
  }

  /**
   * A getter that returns a shared pointer to the Contents object.
   */
  std::shared_ptr<Contents> GetContents() { return contents; }

  /**
   * Make a copy of the message contents.
   */
  void Clone(Work* work) { contents = work->GetContents(); }

  /**
   * \return the number of bytes for the MPI buffer.
   */
  int GetSize() const {
    if (!contents) return 0;
    return contents->GetSize();
  }

  /**
   * Send this work to the node specified by dest.
   * \param dest destination node.
   * \param comm a pointer to the communicator.
   */
  void Send(int dest, Communicator* comm) {
    comm->Send(dest, this);
  }

  /**
   * Send this work to all nodes including the node calling this function.
   * \param comm a pointer to the communicator.
   */
  void SendAll(Communicator* comm) {
    comm->SendAll(this);
  }

  /**
   * Send this work to all nodes including the node calling this function.
   * Same as SendAll.
   * \param comm a pointer to the communicator.
   */
  void SendCollective(Communicator* comm) {
    comm->SendAll(this);
  }

  /**
   * Send this work to all nodes excluding the node calling this function.
   * \param comm a pointer to the communicator.
   */
  void SendAllOther(Communicator* comm) {
    comm->SendAllOther(this);
  }

 protected:
  friend class Communicator;

  /**
   * A getter.
   * \return MPI request object.
   */
  MPI_Request& GetMpiRequest() { return mpiRequest; }

  /**
   * Test if a given message has been sent.
   */
  bool IsSent() {
    int flag;
    MPI_Test(&mpiRequest, &flag, &mpiStatus);
    return flag;
  }

  /**
   * Get a pointer to the MPI buffer.
   */
  template <class T>
  const T* GetBufferPtr() const {
    assert(contents);
    return reinterpret_cast<const T*>(contents->Get());
  }

  /**
   * Get a pointer to the MPI buffer.
   */
  template <class T>
  T* GetBufferPtr() {
    assert(contents);
    return reinterpret_cast<T*>(contents->Get());
  }

  /**
   * Allocate memory for the MPI buffer.
   */
  void Allocate(std::size_t n) {
    assert(!contents);
    assert(n > 0);
    contents.reset(new Contents(n));
  }

  /**
   * A shared pointer to the message contents.
   */
  std::shared_ptr<Contents> contents;

  /**
   * A MPI request object used for nonblocking MPI sends.
   */
  MPI_Request mpiRequest;

  /**
   * A MPI status object.
   */
  MPI_Status mpiStatus;
};

}  // namespace unit
}  // namespace render
}  // namespace gvt

#endif
