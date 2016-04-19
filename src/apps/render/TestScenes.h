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
#ifndef GVT_APPS_RENDER_TEST_SCENES_H
#define GVT_APPS_RENDER_TEST_SCENES_H

#include "gvt/core/DatabaseNode.h"
#include "gvt/render/data/primitives/BBox.h"
#include <string>

namespace gvt {
namespace render {
class RenderContext;
}
}

namespace gvt {
namespace render {
namespace unit {
struct MpiRendererOptions;
}
}
}

namespace apps {
namespace render {

class TestScenes {
public:
  TestScenes(const gvt::render::unit::MpiRendererOptions &options);
  void makePlyDatabase();
  void makeObjDatabase();

private:
  bool isNodeTypeReserved(const std::string &type);
  gvt::core::DBNodeH getNode(const gvt::core::Uuid &id);
  gvt::core::Uuid createNode(const std::string &type, const std::string &name);
  gvt::core::Uuid addMesh(const gvt::core::Uuid &parentNodeId, const std::string &meshName,
                          const std::string &objFilename);
  gvt::render::data::primitives::Box3D getMeshBounds(const gvt::core::Uuid &id);
  gvt::render::data::primitives::Box3D getMeshBounds(const std::string &objFilename);
  gvt::core::Uuid addInstance(const gvt::core::Uuid &parentNodeId, const gvt::core::Uuid &meshId, int instanceId,
                              const std::string &instanceName, glm::mat4 *transform);
  gvt::core::Uuid addPointLight(const gvt::core::Uuid &parentNodeId, const std::string &lightName,
                                const glm::vec3 &position, const glm::vec3 &color);
  // gvt::core::Uuid createCameraNode(const gvt::core::math::Point4f &eye, const gvt::core::math::Point4f &focus,
  //                                  const gvt::core::math::Vector4f &upVector, float fov, unsigned int width,
  //                                  unsigned int height);
  gvt::core::Uuid createFilmNode(int width, int height, const std::string &sceneName);
  gvt::core::Uuid createScheduleNode(int schedulerType, int adapterType);

  gvt::render::RenderContext *renderContext;
  const gvt::render::unit::MpiRendererOptions &options;
  // gvt::render::data::scene::gvtPerspectiveCamera *camera;
};
}
}

#endif
