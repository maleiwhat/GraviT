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

#include <gvt/render/actor/Ray.h>
#include <map>
#include <mutex>
#include <type_traits>

namespace gvt {
namespace tracer {
struct QueuePolicy {
  virtual int policyCheck(const std::map<int, gvt::render::actor::RayVector> &_queue) {
    int id = -1;
    std::size_t _total = 0;
    for (const auto &q : _queue) {
      if (q.second.size() > _total) {
        id = q.first;
        _total = q.second.size();
      }
    }
    return id;
  }
};

struct LargestQueueFirst : public gvt::tracer::QueuePolicy {
  virtual int policyCheck(const std::map<int, gvt::render::actor::RayVector> &_queue) {
    int id = -1;
    std::size_t _total = 0;
    for (const auto &q : _queue) {
      if (q.second.size() > _total) {
        id = q.first;
        _total = q.second.size();
      }
    }
    return id;
  }
};

struct RayQueueManager {

  std::mutex _protect;
  std::map<int, gvt::render::actor::RayVector> _queue;

  std::shared_ptr<QueuePolicy> QP;

  RayQueueManager() { QP = std::make_shared<QueuePolicy>(); };
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
  virtual bool dequeue_send(const int id, gvt::render::actor::RayVector &_raylist) {

    if (_queue.find(id) == _queue.end()) return false;

    std::lock_guard<std::mutex> _lock(_protect);
    std::swap(_queue[id], _raylist);
    _queue.erase(id);
    return true;
  }
  bool empty() {
    std::lock_guard<std::mutex> _lock(_protect);
    if (!_queue.empty()) {
      for (auto q : _queue)
        if (q.second.size() > 0) {
          // std::cout << "Queue " << q.first << " has " << q.second.size() << std::endl
          // << std::flush;
          return false;
        }
    }
    return true;
  }
  std::size_t size() { return _queue.size(); }

  template <class Policy> void setQueuePolicy() {
    static_assert(std::is_base_of<QueuePolicy, Policy>::value,
                  "T must inherit from list");
    QP.reset(new Policy());
  }
  void dequeue(int &target, gvt::render::actor::RayVector &_raylist) {
    int id = -1;
    unsigned _total = 0;
    {
      std::lock_guard<std::mutex> _lock(_protect);
      id = QP->policyCheck(_queue);
    }

    target = id;
    if (id == -1) return;
    std::lock_guard<std::mutex> _lock(_protect);
    std::swap(_queue[id], _raylist);
    _queue.erase(id);
    return;
  }
};

// template <>
// void RayQueueManager::dequeue<HighestSizedQueueFirstPolicy>(
//     int &target, gvt::render::actor::RayVector &);
}
}

#endif /* GVT_RAYQUEUEMANAGER_H */
