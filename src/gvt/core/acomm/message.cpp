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
#include <gvt/core/acomm/message.h>

namespace gvt {
namespace comm {

MESSAGE_HEADER_INIT(Message);

Message::Message() {}

Message::Message(const size_t &size) : _size(size) {
  _buffer = make_shared_buffer<unsigned char>(size);
  // std::make_shared<unsigned char>(new unsigned char[size], std::default_delete<unsigned
  // char[]>());
  GVT_ASSERT(_buffer, "Error allocation message buffer");
}

Message::Message(const void *message, const size_t &size) : Message(_size) {
  std::memcpy(_buffer.get(), message, _size);
}

Message::Message(const Message &other) : Message(other._buffer.get(), other._size) {
  std::memcpy(_buffer.get(), other._buffer.get(), _size);
}

Message::Message(Message &&other) {
  std::swap(_buffer, other._buffer);
  _size = other._size;
}

bool Message::sent() { return _sent; }
bool Message::wait() {
  std::unique_lock<std::mutex> lk(m);
  cv.wait(lk, [&] { return this->sent(); });
  lk.unlock();
  return true;
}

void Message::setcontent(const void *_data, const size_t &size) {
  GVT_ASSERT(size <= _size, "Wrong size buffer");
  std::memcpy(_buffer.get(), _data, _size);
}

// virtual void Message::sendto(size_t target, bool wait) {
//   std::shared_ptr<acommunicator> comm = gvt::comm::acommunicator::instance();
//
//   comm->
//
// }
// virtual void Message::broadcast(bool wait) {}
}
}
