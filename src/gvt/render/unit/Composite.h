#ifndef GVT_RENDER_UNIT_COMPOSITE_H
#define GVT_RENDER_UNIT_COMPOSITE_H

#include <IceT.h>
#include <glm/glm.hpp>

namespace gvt {
namespace render {
namespace unit {

class Composite {
 public:
  // Composite() {}
  // void Init(int width, int height);
  // glm::vec4 *Execute(int width, int height, glm::vec4 *buffer_in);

  // glm::vec4 *buffer;
  IceTInt num_proc;
  Composite() {}
  ~Composite() {}
  bool InitIceT();
  glm::vec4 *Execute(glm::vec4 *buffer_in, const size_t width, const size_t height);

//  private:
//   int width, height;
};

}  // namespace unit
}  // namespace render
}  // namespace gvt

#endif
