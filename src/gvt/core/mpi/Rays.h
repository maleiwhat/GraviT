#pragma once

#include "Work.h"

namespace gvt {
namespace core {
namespace mpi {

class Ray {
  float x, y, z;
  float dx, dy, dz;
};

class Rays : public Work {
public:
  WORK_CLASS(Rays, true);
  bool Action();

  int size() { return contents->get_size() / sizeof(Ray); }
  Ray *get() { return (Ray *)contents->get(); }
};
}
}
}
