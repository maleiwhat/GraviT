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

#ifndef GVT_IMAGETRACER_H
#define GVT_IMAGETRACER_H

#include <map>
#include <memory>
#include <vector>

#include <gvt/core/Debug.h>
#include <gvt/core/tracer/tracer.h>

#include <gvt/render/Adapter.h>
#include <gvt/render/actor/Ray.h>
#include <gvt/render/data/accel/AbstractAccel.h>

#include <gvt/render/tracer/RayQueueManager.h>

#include <gvt/render/Adapter.h>
#include <gvt/render/adapter/AdapterCache.h>
namespace gvt {
namespace tracer {

class ImageTracer : public gvt::tracer::Tracer {
protected:
public:
  ImageTracer();
  virtual ~ImageTracer();
  virtual void operator()();
  virtual bool MessageManager(std::shared_ptr<gvt::comm::Message> msg);

  virtual void processRayQueue(gvt::render::actor::RayVector &rays, const int src = -1,
                               const int dst = -1);

  virtual void updateGeometry();

private:
  RayQueueManager _queue;
  gvt::render::actor::RayVector _rayqueue;
  gvt::render::AdapterCache<int, std::shared_ptr<gvt::render::Adapter> > _cache;
  std::shared_ptr<gvt::render::data::accel::AbstractAccel> global_bvh = nullptr;

  // std::map<int, gvt::render::data::primitives::Mesh *> meshRef;
};

template <>
void RayQueueManager::dequeue<ImageTracer>(int &target, gvt::render::actor::RayVector &);
}
}

#endif
