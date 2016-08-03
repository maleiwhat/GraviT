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

#ifndef GVT_CORE_ACOMMUNICATOR_H
#define GVT_CORE_ACOMMUNICATOR_H

#include <deque>
#include <memory>
#include <mutex>

#include <gvt/core/acomm/message.h>

namespace gvt {
namespace comm {
class acommunicator {
protected:
  acommunicator(acommunicator &comm) { GVT_ASSERT(false, "Communicator cannot be copied"); }

  enum COMM_TAG { COMMUNICATOR_CONTROL, USER_DEFINED_MSG, VOTE_MSG_TAG };

public:
  static std::shared_ptr<acommunicator> _singleton;
  static std::mutex m;
  acommunicator();
  ~acommunicator();

  static std::shared_ptr<acommunicator> instance();

  size_t id() const;
  size_t maxid() const;

  void send(std::shared_ptr<Message> msg, size_t target, bool sync = false);
  void broadcast(std::shared_ptr<Message> msg, bool sync = false);

  void terminate() { _terminate = true; }

  void run(acommunicator *com);

private:
  acommunicator(const acommunicator &) = delete;
  acommunicator &operator=(const acommunicator &) = delete;

  std::deque<std::shared_ptr<Message> > _outbox;
  std::deque<std::shared_ptr<Message> > _inbox;

  bool _terminate = true;
};
}
}

#endif /*GVT_CORE_ACOMMUNICATOR_H*/
