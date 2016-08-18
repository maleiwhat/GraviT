// clang-format off
/*
  =======================================================================================
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
   =======================================================================================

   */
// clang-format on

#include <memory>

#include <gvt/render/Context.h>
#include "ImageTracer.h"

namespace gvt {
namespace tracer {

template <>
gvt::render::actor::RayVector &&RayQueueManager::dequeue<ImageTracer>(int &target) {
  int id = -1;
  unsigned _total = 0;
  {
    std::lock_guard<std::mutex> _lock(_protect);
    for (const auto &q : _queue) {
      if (q.second.size() > _total) {
        id = q.first;
        _total = q.second.size();
      }
    }
  }

  if (id == -1) return std::move(gvt::render::actor::RayVector());
  gvt::render::actor::RayVector _raylist = dequeue(id);

  std::lock_guard<std::mutex> _lock(_protect);
  _queue.erase(id);
  return std::move(_raylist);

  // return (id != -1) ? dequeue(id) : gvt::render::actor::RayVector();
}

ImageTracer::ImageTracer() : Tracer() {}

ImageTracer::~ImageTracer() {}

void ImageTracer::operator()() {

  bool GlobalFrameFinished = false;
  gvt::render::RenderContext *cntx = gvt::render::RenderContext::instance();
  while (!GlobalFrameFinished) {
    if (!_queues.empty()) {
      int target;
      gvt::render::actor::RayVector toprocess = _queues.dequeue<ImageTracer>(target);
      if (target != -1) {
        // Check cache if adpter exists;
        // Call adapter
        // Process rays
      }
    }
    if (_queues.empty()) {
      // Ask if done;
    }
  }
  // Start composite
};

bool ImageTracer::MessageManager(std::shared_ptr<gvt::comm::Message> msg) {
  return Tracer::MessageManager(msg);
}

void ImageTracer::processRayQueue(gvt::render::actor::RayVector &rays, const int src,
                                  const int dst) {}

void ImageTracer::updateGeometry() {}
}
}
