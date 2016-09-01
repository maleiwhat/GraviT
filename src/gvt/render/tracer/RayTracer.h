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

#ifndef GVT_RAYTRACER_H
#define GVT_RAYTRACER_H

#include <map>
#include <memory>
#include <vector>

#include <gvt/core/Debug.h>
#include <gvt/core/tracer/tracer.h>

#include <gvt/render/Adapter.h>
#include <gvt/render/actor/Ray.h>
#include <gvt/render/data/accel/AbstractAccel.h>

#include <gvt/render/tracer/QueueManager/RayQueueManager.h>

#include <gvt/render/Adapter.h>
#include <gvt/render/adapter/AdapterCache.h>
namespace gvt {
namespace tracer {

class RayTracer : public gvt::tracer::Tracer {
protected:
  RayQueueManager _queue;
  // gvt::render::actor::RayVector _rayqueue;
  gvt::render::AdapterCache<int, std::shared_ptr<gvt::render::Adapter> > _cache;
  std::shared_ptr<gvt::render::data::accel::AbstractAccel> global_bvh = nullptr;

public:
  RayTracer() : Tracer(){};
  virtual ~RayTracer(){};
  virtual void operator()() = 0;
  virtual bool MessageManager(std::shared_ptr<gvt::comm::Message> msg) = 0;

  virtual void processRayQueue(gvt::render::actor::RayVector &rays, const int src = -1,
                               const int dst = -1);

  virtual void updateGeometry() = 0;

  virtual void trace(int target, gvt::render::data::primitives::Mesh *mesh,
                     gvt::render::actor::RayVector &rayList,
                     gvt::render::actor::RayVector &moved_rays, glm::mat4 *m,
                     glm::mat4 *minv, glm::mat3 *minvn,
                     std::vector<gvt::render::data::scene::Light *> &lights,
                     size_t begin = 0, size_t end = 0);

private:
};
}
}

#endif
