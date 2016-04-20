#ifndef GVT_CORE_MPI_RENDERTILE_H
#define GVT_CORE_MPI_RENDERTILE_H

#include <gvt/core/mpi/RenderWork.h>

namespace gvt {
namespace core {
namespace mpi {
struct RenderTile {};

class RenderTileWork : public RenderWork {
public:
  virtual bool Action() { return true; };

private:
  struct RenderTile renderTile;
  virtual void GenerateRaysFromTiles();
};

} // ns mpi
} // ns core
} // ns gvt

#endif /* GVT_CORE_MPI_RENDERTILE_H */
