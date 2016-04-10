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
class MpiRendererOptions;
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
                              const std::string &instanceName,
                              gvt::core::math::AffineTransformMatrix<float> *transform);
  gvt::core::Uuid addPointLight(const gvt::core::Uuid &parentNodeId, const std::string &lightName,
                                const gvt::core::math::Vector4f &position, const gvt::core::math::Vector4f &color);
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
