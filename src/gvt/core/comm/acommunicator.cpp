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

   GraviT is funded in part by the US National Science Foundation under awards
   ACI-1339863,
   ACI-1339881 and ACI-1339840
   =======================================================================================
   */

#include <gvt/core/Debug.h>
#include <gvt/core/comm/acommunicator.h>
#include <gvt/core/comm/message.h>

#include <mpi.h>
#include <thread>

#include <iostream>

namespace gvt {
namespace comm {

std::shared_ptr<acommunicator> acommunicator::_singleton = nullptr;
tbb::task_group acommunicator::g;

std::mutex acommunicator::m;

std::shared_ptr<acommunicator> acommunicator::instance(int argc, char *argv[]) {

  std::lock_guard<std::mutex> lg(acommunicator::m);
  if (acommunicator::_singleton == nullptr) {
    acommunicator::_singleton =
        std::shared_ptr<acommunicator>(new acommunicator(argc, argv));
    g.run([&]() { acommunicator::launch(acommunicator::_singleton); });
  }

  return acommunicator::_singleton;
}

acommunicator::acommunicator(int argc, char *argv[]) {
  // MPI_Init(&argc, &argv);
  _terminate = false;
  MPI::Init_thread(argc, argv, MPI_THREAD_MULTIPLE);
}
acommunicator::~acommunicator() {
  MPI::Finalize();
  // MPI_Finalize();
}

void acommunicator::terminate() {
  // MPI::COMM_WORLD.Barrier();
  _terminate = true;
  g.wait();
  // std::cout << "Terminate comm channel : " << id() << std::endl;
  // MPI::Finalize();
  // MPI_Finalize();
}

bool acommunicator::hasMessages() { return _inbox.size() > 0; }
bool acommunicator::hasAnyMessages() { return _inbox.size() > 0 || _outbox.size() > 0; }

size_t acommunicator::id() const { return MPI::COMM_WORLD.Get_rank(); }
size_t acommunicator::maxid() const { return MPI::COMM_WORLD.Get_size(); }

void acommunicator::send(std::shared_ptr<Message> msg, size_t target, bool sync) {
  GVT_ASSERT(target < maxid(), "Trying to send a message to a bad processor ID");
  msg->_dst = target;
  msg->msg_tag() = msg->msg_static_tag();

  if (sync) {
    MPI::COMM_WORLD.Isend(msg->msg_ptr(), msg->msg_size(), MPI::UNSIGNED_CHAR, target,
                          USER_DEFINED_MSG);
  } else {
    _outbox.push_back(msg);
  }
}

void acommunicator::broadcast(std::shared_ptr<Message> msg, bool sync) {
  msg->_dst = -1;
  msg->msg_tag() = msg->msg_static_tag();
  if (sync) {
    MPI::COMM_WORLD.Bcast(msg->msg_ptr(), msg->msg_size(), MPI::UNSIGNED_CHAR, 0);
  } else {
    std::lock_guard<std::mutex> lk(_outbox_mutex);
    _outbox.push_back(msg);
  }
}

std::shared_ptr<Message> acommunicator::popMessage() {
  std::lock_guard<std::mutex> lk(_inbox_mutex);
  if (_inbox.empty()) return std::make_shared<Message>();
  std::shared_ptr<Message> msg = _inbox.front();
  _inbox.pop_front();
  return msg;
}

void acommunicator::launch(std::shared_ptr<acommunicator> com) { com->run(); }

void acommunicator::run() {

  while ((!_outbox.empty() || !_inbox.empty()) || !_terminate) {
    if (!_outbox.empty()) {
      std::lock_guard<std::mutex> lk(_outbox_mutex);
      std::shared_ptr<Message> msg = _outbox.front();
      _outbox.pop_front();
      if (msg->dst() == -1) {
        MPI::COMM_WORLD.Bcast(msg->msg_ptr(), msg->msg_size(), MPI::UNSIGNED_CHAR, 0);
        // std::cout << "Broadcasted message on channel " << id() << std::endl;
      } else {
        GVT_ASSERT(msg->dst() < maxid(),
                   "Trying to send a message to a bad processor ID");
        MPI::COMM_WORLD.Isend(msg->msg_ptr(), msg->msg_size(), MPI::UNSIGNED_CHAR,
                              msg->dst(), USER_DEFINED_MSG);
      }
    }

    MPI::Status status;
    if (MPI::COMM_WORLD.Iprobe(MPI::ANY_SOURCE, COMMUNICATOR_CONTROL, status)) {
      // std::cout << "Got comm messages : " << id() << std::endl;
      const auto sender = status.Get_source();
      const auto n_bytes = status.Get_count(MPI::BYTE);

      if (n_bytes > 0) {
        std::shared_ptr<Message> msg = std::make_shared<Message>(n_bytes - sizeof(long));
        MPI::COMM_WORLD.Recv(msg->msg_ptr(), n_bytes, MPI::UNSIGNED_CHAR, sender,
                             COMMUNICATOR_CONTROL);
      }
    }

    // Check for messages and put on the inbox

    if (MPI::COMM_WORLD.Iprobe(MPI::ANY_SOURCE, USER_DEFINED_MSG, status)) {
      const auto sender = status.Get_source();
      const auto n_bytes = status.Get_count(MPI::BYTE);
      // std::cout << "Get user messages " << id() << std::endl;
      if (n_bytes > 0) {
        std::shared_ptr<Message> msg = std::make_shared<Message>(n_bytes - sizeof(long));
        MPI::COMM_WORLD.Recv(msg->msg_ptr(), n_bytes, MPI::UNSIGNED_CHAR, sender,
                             USER_DEFINED_MSG);
        std::lock_guard<std::mutex> lk(_inbox_mutex);
        _inbox.push_back(msg);
      }
    }

    if (MPI::COMM_WORLD.Iprobe(MPI::ANY_SOURCE, VOTE_MSG_TAG, status)) {
      // std::cout << "Got vote messages " << id() << std::endl;
      const auto sender = status.Get_source();
      const auto n_bytes = status.Get_count(MPI::BYTE);

      if (n_bytes > 0) {
        std::shared_ptr<Message> msg = std::make_shared<Message>(n_bytes - sizeof(long));
        MPI::COMM_WORLD.Recv(msg->msg_ptr(), n_bytes, MPI::UNSIGNED_CHAR, sender,
                             VOTE_MSG_TAG);
      }
    }
  }
}
}
}
