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

#ifndef GVT_CORE_MESSAGE_H
#define GVT_CORE_MESSAGE_H

#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>

namespace gvt {
namespace comm {

template <typename T> std::shared_ptr<T> make_shared_buffer(size_t size) {
  return std::shared_ptr<T>(new T[size], std::default_delete<T[]>());
}

#define MESSAGE_HEADER(ClassName)                                                        \
public:                                                                                  \
  static long getTag() { return ClassName::tag; }                                        \
  static void setTag(const long _tag) { ClassName::tag = _tag; }                         \
  virtual std::string getNameTag() { return #ClassName; }                                \
  virtual long msg_static_tag() { return ClassName::tag; }                               \
                                                                                         \
private:                                                                                 \
  static long tag;                                                                       \
  static std::string tagName

#define MESSAGE_HEADER_INIT(ClassName)                                                   \
  long ClassName::tag = -1;                                                              \
  std::string ClassName::tagName = #ClassName;

class Message {
  MESSAGE_HEADER(Message);

protected:
  std::shared_ptr<unsigned char> _buffer = nullptr;
  size_t _size = 0;

  std::mutex m;
  std::condition_variable cv;

  bool _sent = false;

public:
  Message();
  Message(const size_t &size);
  Message(const void *content, const size_t &size);
  Message(const Message &other);
  Message(Message &&other);

  template <class U> operator U *() {
    static_assert(std::is_base_of<gvt::comm::Message, U>::value,
                  "U must inherit from gvt::comm::Message");
    return dynamic_cast<U>(this);
  }

  virtual void setcontent(const void *_data, const size_t &size);

  bool sent();
  bool wait();

  // virtual void sendto(size_t dst, bool wait = false);
  // virtual void broadcast(bool wait = false);

  inline void *msg_ptr() { return (void *)(_buffer.get()); };
  inline long &msg_tag() { return *(long *)(_buffer.get() + _size); };
  inline size_t msg_size() { return _size; };
  inline size_t size() { return _size - sizeof(long); }
  inline size_t dst() { return _dst; }

  // TO BE USED BY COMMUNICATOR
  size_t _dst = -1;
};
}
}

#endif /* GVT_CORE_MESSAGE_H */
