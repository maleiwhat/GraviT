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

#ifndef GVT_RAYQUEUEMANAGER_H
#define GVT_RAYQUEUEMANAGER_H

namespace gvt {
namespace tracer {
struct RayQueueManager {

  std::mutex _protect;

  std::map<int, gvt::render::actor::RayVector> _queue;

  RayQueueManager(){};

  ~RayQueueManager() {}

  virtual void enqueue(int id, gvt::render::actor::RayVector &_raylist) {
    std::lock_guard<std::mutex> _lock(_protect);
    if (_queue.find(id) == _queue.end()) {
      _queue[id] = gvt::render::actor::RayVector();
    }
    if (_queue[id].size() == 0)
      std::swap(_queue[id], _raylist);
    else {
      _queue[id].insert(_queue[id].end(), std::make_move_iterator(_raylist.begin()),
                        std::make_move_iterator(_raylist.end()));
    }
  }

  virtual gvt::render::actor::RayVector dequeue(int id) {
    GVT_ASSERT(_queue.find(id) != _queue.end(), "Trying to access an invalid queue ["
                                                    << id << "]");
    std::lock_guard<std::mutex> _lock(_protect);
    return gvt::render::actor::RayVector();
  }

  bool empty() { return _queue.empty(); }
  std::size_t size() { return _queue.size(); }

  template <class scheduler> void dequeue(int &target, gvt::render::actor::RayVector &) {
    GVT_ASSERT(false, "Queue policity not defined");
  }
};
}
}

#endif /* GVT_RAYQUEUEMANAGER_H */
