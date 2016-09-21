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

#ifndef GVT_TRACER_H
#define GVT_TRACER_H

#include <gvt/core/Debug.h>
#include <gvt/core/comm/comm.h>
#include <map>
#include <string>
#include <type_traits>
#include <vector>

namespace gvt {
namespace tracer {
class Tracer {
protected:
  std::vector<std::string> _registered_messages;

public:
  Tracer() {}
  virtual ~Tracer() {}
  virtual void operator()() { GVT_ASSERT(false, "Tracer not implemented"); };

  virtual bool MessageManager(std::shared_ptr<gvt::comm::Message> msg) {
    std::cout << "aaa" << std::endl;
    return false;
  }

  template <class M> int RegisterMessage() {
    // static_assert(std::is_base_of<gvt::comm::Message, M>::value,
    //               "M must inherit from gvt::comm::Message");
    //
    // M msg;
    // _registered_messages.push_back(msg.getNameTag());
    // M::setTag(_registered_messages.size() - 1);
    // return M::getTag();
    return gvt::comm::communicator::RegisterMessageType<M>();
  }

  virtual bool isDone() {
    GVT_ASSERT(false, "Tracer not implemented");
    return false;
  }

  virtual bool hasWork() {
    GVT_ASSERT(false, "Tracer not implemented");
    return false;
  }

  virtual float *getImageBuffer() {
    GVT_ASSERT(false, "Tracer not implemented");
    return nullptr;
  };

  // template <class M> static int RegisterMessageType() {
  //   static_assert(std::is_base_of<comm::Message, M>::value,
  //                 "M must inherit from comm::Message");
  //   std::string classname = typeid(M).name();
  //   if (registry_ids.find(classname) != registry_ids.end()) {
  //     return registry_ids[classname];
  //   }
  //   registry_names.push_back(classname);
  //   std::size_t idx = registry_names.size() - 1;
  //   registry_ids[classname] = idx;
  //   M::COMMUNICATOR_MESSAGE_TAG = idx;
  //   return idx;
  // }
  //
  // template <class M>
  // static std::shared_ptr<M> SAFE_DOWN_CAST(const std::shared_ptr<comm::Message> &msg) {
  //   if (msg->tag() >= registry_names.size()) return nullptr;
  //   std::string classname = registry_names[msg->tag()];
  //   if (registry_ids.find(classname) == registry_ids.end()) return nullptr;
  //   if (registry_ids[classname] == msg->tag()) return std::static_pointer_cast<M>(msg);
  //   return nullptr;
  // }
  //
  // static std::vector<std::string> registry_names;
  // static std::map<std::string, std::size_t> registry_ids;

private:
};
}
}

#endif
