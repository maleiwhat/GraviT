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
// RenderWork.h
//

#ifndef GVT_CORE_MPI_RENDER_WORK_H
#define GVT_CORE_MPI_RENDER_WORK_H

#include "gvt/core/mpi/Work.h"
#include "gvt/render/actor/Ray.h"
#include "gvt/core/mpi/Application.h"

using namespace std;
using namespace gvt::core::mpi;

namespace gvt {
namespace core {
namespace mpi {

class RenderWork : public Work {
public:
  virtual bool Action() {return true;};
private:
  int uniqueId; 
  gvt::render::actor::RayVector rays;
};

class TileWork : public Work {
  WORK_CLASS_HEADER(TileWork)
public:
  virtual void intialize();
  virtual ~TileWork() {}
  virtual void Serialize(size_t& size, unsigned char *& serialized);
  static Work* Deserialize(size_t size, unsigned char* serialized);
  virtual bool Action();
protected:
  int startX;
  int startY;
  int width;
  int height;
};

class RayWork : public Work {
  WORK_CLASS_HEADER(RayWork)
public:
  // virtual void intialize();
  virtual ~RayWork() {}
  // static Work* Deserialize(size_t size, unsigned char* serialized);
  // virtual bool Action();
protected:
  gvt::render::actor::RayVector rays;
};

// TODO: define this for the image and hybrid schedulers
class GeometryWork : public Work {
  WORK_CLASS_HEADER(GeometryWork)
public:
  virtual void intialize() {}
  virtual ~GeometryWork() {}
  static Work* Deserialize(size_t size, unsigned char* serialized);
  virtual bool Action() { return true; }
};

class PixelWork : public Work {
  WORK_CLASS_HEADER(PixelWork)
public:
  // virtual void intialize() {}
  virtual ~PixelWork() {}
  // static Work* Deserialize(size_t size, unsigned char* serialized);
  // virtual bool Action();
// protected:
};

} //ns mpi
} //ns core
} //ns gvt


#endif /* GVT_CORE_MPI_RENDER_H */
