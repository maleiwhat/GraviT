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
// MpiRenderer.h
//

#ifndef GVT_RENDER_UNIT_MPI_RENDERER_H
#define GVT_RENDER_UNIT_MPI_RENDERER_H

#include "gvt/core/mpi/Application.h"
#include "gvt/core/DatabaseNode.h"
#include "gvt/core/math/Vector.h"
#include "gvt/core/Types.h"

#include "gvt/render/Types.h"
#include "gvt/render/data/primitives/BBox.h"

#include <vector>
#include <tbb/mutex.h>
   
using namespace gvt::core::mpi;

namespace gvt { namespace render {
  class RenderContext;
}}

namespace gvt { namespace render { namespace data { namespace scene {
  class gvtPerspectiveCamera;
  class Image;
}}}}

namespace gvt { namespace render { namespace data { namespace accel {
  class AbstractAccel;
}}}}

namespace gvt {
namespace render {
namespace unit {

namespace rank { enum RankType { Server=0, Display=1 }; }

class TileLoadBalancer;

// TODO (hpark): make a command line parser using ConfigFileLoader
//  We could utilize existing ConfigFileLoader
//  instead of the following option structs.
//  The following will suffice for now.

struct DatabaseOption {
  virtual ~DatabaseOption() {}
  int schedulerType = gvt::render::scheduler::Domain;
  int adapterType = gvt::render::adapter::Embree;
  // std::vector<std::string> meshFilenames; // TODO
};

struct TestDatabaseOption : public DatabaseOption {
  int instanceCountX = 2;
  int instanceCountY = 2;
  int instanceCountZ = 1;
};

class MpiRenderer : public Application {
public:
  MpiRenderer(int *argc, char ***argv);
  virtual ~MpiRenderer();

  // for configuring database

  virtual void parseCommandLine(int argc, char** argv);
  virtual void createDatabase();
  
  // helper APIs

  bool isNodeTypeReserved(const std::string& type);

  gvt::core::DBNodeH getNode(const gvt::core::Uuid& id);

  gvt::core::Uuid createNode(const std::string& type,
                             const std::string& name);

  gvt::core::Uuid addMesh(const gvt::core::Uuid& parentNodeId,
                          const std::string& meshName,
                          const std::string& objFilename);

  gvt::render::data::primitives::Box3D
    getMeshBounds(const gvt::core::Uuid& id);

  gvt::core::Uuid
    addInstance(const gvt::core::Uuid& parentNodeId,
                const gvt::core::Uuid& meshId,
                int instanceId,
                const std::string& instanceName,
                gvt::core::math::AffineTransformMatrix<float>* transform);

  gvt::core::Uuid addPointLight(const gvt::core::Uuid& parentNodeId,
                     const std::string& lightName,
                     const gvt::core::math::Vector4f& position,
                     const gvt::core::math::Vector4f& color);

  gvt::core::Uuid createCameraNode(const gvt::core::math::Point4f& eye,
                        const gvt::core::math::Point4f& focus,
                        const gvt::core::math::Vector4f& upVector,
                        float fov,
                        unsigned int width, 
                        unsigned int height);

  gvt::core::Uuid createFilmNode(int width,
                      int height,
                      const std::string& sceneName);

  gvt::core::Uuid createScheduleNode(int schedulerType, int adapterType);
  void render();

public:
  TileLoadBalancer* getTileLoadBalancer() { return tileLoadBalancer; }
  gvt::render::RenderContext* getRenderContext() { return renderContext; }
  const gvt::render::data::scene::gvtPerspectiveCamera* getCamera() const { return camera; }
  gvt::render::data::scene::Image* getImage() { return image; }
  std::vector<GVT_COLOR_ACCUM>* getFramebuffer() { return &framebuffer; }

  int decrementPendingPixelCount(int amount) {
    pendingPixelCount -= amount;
    return pendingPixelCount;
  }

  gvt::core::Vector<gvt::core::DBNodeH>& getInstanceNodes() { return instanceNodes; }
  gvt::render::data::accel::AbstractAccel* getAcceleration() { return acceleration; }
  tbb::mutex* getQueueMutex() { return queue_mutex; }
  tbb::mutex* getColorBufMutex() { return colorBuf_mutex; }

private:
  void initServer();
  void initDisplay();
  void initWorker();
  void setupRender();
  void freeRender();

private:
  DatabaseOption* dbOption;
  gvt::render::RenderContext* renderContext;
  gvt::core::Vector<gvt::core::DBNodeH> instanceNodes;
  gvt::core::DBNodeH root;

  gvt::render::data::scene::gvtPerspectiveCamera* camera;
  gvt::render::data::scene::Image* image;
  std::vector<GVT_COLOR_ACCUM> framebuffer;
  int pendingPixelCount;
  int imageWidth;
  int imageHeight;

  TileLoadBalancer* tileLoadBalancer;
  gvt::render::data::accel::AbstractAccel* acceleration;
  tbb::mutex* queue_mutex;
  tbb::mutex* colorBuf_mutex;
};

}
}
}

#endif
