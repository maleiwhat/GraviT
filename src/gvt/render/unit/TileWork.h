/* =======================================================================================
   This file is released as part of GraviT - scalable, platform independent ray
   tracing
   tacc.github.io/GraviT

   Copyright 2013-2015 Texas Advanced Computing Center, The University of Texas
   at Austin
   All rights reserved.

   Licensed under the BSD 3-Clause License, (the "License"); you may not use
   this file
   except in compliance with the License.
   A copy of the License is included with this software in the file LICENSE.
   If your copy does not contain the License, you may obtain a copy of the
   License at:

       http://opensource.org/licenses/BSD-3-Clause

   Unless required by applicable law or agreed to in writing, software
   distributed under
   the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY
   KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under
   limitations under the License.

   GraviT is funded in part by the US National Science Foundation under awards
   ACI-1339863,
   ACI-1339881 and ACI-1339840
   =======================================================================================
   */

//
// TileWork.h
//

#ifndef GVT_RENDER_UNIT_TILE_WORK_H
#define GVT_RENDER_UNIT_TILE_WORK_H

#include "gvt/core/Types.h"
#include "gvt/core/DatabaseNode.h"
#include "gvt/core/mpi/Work.h"
#include "gvt/core/mpi/Application.h"
   
#include "gvt/render/data/scene/ColorAccumulator.h"
#include "gvt/render/actor/Ray.h"

#include <vector>
#include <map>

#include <tbb/mutex.h>

using namespace gvt::core::mpi;
using namespace gvt::render::actor;

namespace gvt { namespace render { namespace data { namespace accel {
  class AbstractAccel;
}}}}

namespace gvt { namespace render {
  class Adapter;
}}

namespace gvt {
namespace render {
namespace unit {

class MpiRenderer;

class TileWork : public Work {
  WORK_CLASS_HEADER(TileWork)
public:
  virtual void initialize() { width = -1; } // for validity check
  virtual ~TileWork() {}
  virtual void Serialize(std::size_t& size, unsigned char*& serialized);
  static Work* Deserialize(std::size_t size, unsigned char* serialized);
  virtual bool Action();
  void setTileInfo(int x, int y, int w, int h,
                   std::vector<GVT_COLOR_ACCUM>* framebuffer = NULL);
  void setTileSize(int x, int y, int w, int h);
  void setFramebuffer(std::vector<GVT_COLOR_ACCUM>* fb) { framebuffer = fb; }
  bool isValid() { return (width > 0); }
  int getStartX() const { return startX; }
  int getStartY() const  { return startY; }
  int getWidth() const { return width; }
  int getHeight() const { return height; }
protected:
  virtual void setupAction();
  virtual void generatePrimaryRays(gvt::render::actor::RayVector& rays);
  virtual void traceRays(gvt::render::actor::RayVector& rays);
  virtual void sendRequest(int rank);
  virtual void sendPixels(int rank);
protected: 
  virtual void filterRaysLocally(gvt::render::actor::RayVector& rays);
  virtual void shuffleRays(gvt::render::actor::RayVector &rays,
                           gvt::core::DBNodeH instNode);
  void renderMosaic();
protected:
  MpiRenderer* renderer;
  gvt::core::DBNodeH root;
  int imageWidth;
  int imageHeight;
  gvt::render::data::accel::AbstractAccel* acceleration;
  std::map<int, gvt::render::actor::RayVector> queue;
  std::map<gvt::core::Uuid, gvt::render::Adapter*> adapterCache;
  tbb::mutex* queue_mutex;
  tbb::mutex* colorBuf_mutex;
protected:
  int startX;
  int startY;
  int width;
  int height;
  std::vector<GVT_COLOR_ACCUM>* framebuffer;
};

}
}
}

#endif
