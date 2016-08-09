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

#include <tbb/task_group.h>

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
  acommunicator(int argc, char *argv[]);
  ~acommunicator();

  static std::shared_ptr<acommunicator> instance(int argc = 0, char *argv[] = nullptr);
  static void launch(std::shared_ptr<acommunicator> com);

  size_t id() const;
  size_t maxid() const;

  void send(std::shared_ptr<Message> msg, size_t target, bool sync = false);
  void broadcast(std::shared_ptr<Message> msg, bool sync = false);

  void terminate();

  void run();

  bool hasMessages();

  std::shared_ptr<Message> popMessage();

private:
  acommunicator(const acommunicator &) = delete;
  acommunicator &operator=(const acommunicator &) = delete;

  std::mutex _outbox_mutex;
  std::deque<std::shared_ptr<Message> > _outbox;
  std::mutex _inbox_mutex;
  std::deque<std::shared_ptr<Message> > _inbox;

  static tbb::task_group g;

  bool _terminate = false;
};
}
}

#endif /*GVT_CORE_ACOMMUNICATOR_H*/
