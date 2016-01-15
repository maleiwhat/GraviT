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

#ifndef APPS_RENDER_MPI_RENDERER_APP_H
#define APPS_RENDER_MPI_RENDERER_APP_H

#include "gvt/core/mpi/Application.h"
#include "gvt/core/DatabaseNode.h"
#include "gvt/core/Types.h"
#include "gvt/core/math/Vector.h"

#include "gvt/render/Types.h"
#include "gvt/render/RenderContext.h"
#include "gvt/render/data/scene/gvtCamera.h"
#include "gvt/render/data/scene/Image.h"
#include "gvt/render/data/primitives/BBox.h"
#include "gvt/render/unit/TileLoadBalancer.h"
   
using namespace gvt::core;
using namespace gvt::core::math;
using namespace gvt::core::mpi;
using namespace gvt::render;
using namespace gvt::render::data::scene;
using namespace gvt::render::data::primitives;
using namespace gvt::render::unit;

namespace apps {
namespace render {

namespace rank { enum RankType { Server=0, Display=1 }; }

// hpark TODO:
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

  DBNodeH getNode(const Uuid& id);

  Uuid createNode(const std::string& type,
                  const std::string& name);

  Uuid addMesh(const Uuid& parentNodeId,
               const std::string& meshName,
               const std::string& objFilename);

  Box3D getMeshBounds(const Uuid& id);

  Uuid addInstance(const Uuid& parentNodeId,
                   const Uuid& meshId,
                   int instanceId,
                   const std::string& instanceName,
                   gvt::core::math::AffineTransformMatrix<float>* transform);

  Uuid addPointLight(const Uuid& parentNodeId,
                     const std::string& lightName,
                     const Vector4f& position,
                     const Vector4f& color);

  Uuid createCameraNode(const Point4f& eye,
                        const Point4f& focus,
                        const Vector4f& upVector,
                        float fov,
                        unsigned int width, 
                        unsigned int height);

  Uuid createFilmNode(int width,
                      int height,
                      const std::string& sceneName);

  Uuid createScheduleNode(int schedulerType, int adapterType);
  void render();

  TileLoadBalancer* getTileLoadBalancer() { return tileLoadBalancer; }
  RenderContext* getRenderContext() { return renderContext; }

private:
  void launchTileLoadBalancer();
  void requestWorkToServer();

protected:
  RenderContext* renderContext;
  gvtPerspectiveCamera* camera;
  Image* image;
  DatabaseOption* dbOption;
  TileLoadBalancer* tileLoadBalancer;
};

}
}

#endif
