
#ifndef Manta_DomBox_h
#define Manta_DomBox_h


#include <Model/Primitives/PrimitiveCommon.h>
#include <Core/Geometry/Vector.h>
#include <Core/Geometry/BBox.h>

namespace Manta
{

  class DomBox : public PrimitiveCommon {
  public:
    DomBox(Material* mat, const Vector& min_, const Vector& max_, int domID);
    ~DomBox();
    void setMinMax(const Vector&  p0, const Vector& p1);
    virtual void computeBounds(const PreprocessContext& context,
                               BBox& bbox) const;
    virtual void intersect(const RenderContext& context, RayPacket& rays) const ;
    virtual void computeNormal(const RenderContext& context, RayPacket &rays) const;    
    int domID;
    
  private:
    BBox bbox;
  };
}

#endif
