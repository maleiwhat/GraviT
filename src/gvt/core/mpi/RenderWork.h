#ifndef GVT_CORE_MPI_RENDER_H
#define GVT_CORE_MPI_RENDER_H

#include <gvt/core/mpi/Work.h>
#include <gvt/render/actor/Ray.h>

namespace gvt {
namespace core {
namespace mpi {

class RenderWork : public Work
{

private:
  gvt::render::actor::RayVector rays;

};

} //ns mpi
} //ns core
} //ns gvt


#endif /* GVT_CORE_MPI_RENDER_H */
