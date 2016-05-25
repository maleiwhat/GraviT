#include "Application.h"
#include "Rays.h"
#include "RayQManager.h"

using namespace gvt::core::mpi;

WORK_CLASS_TYPE(Rays);

bool Rays::Action() {
  RayQManager::GetTheRayQManager()->Enqueue(this);
  return false;
}
