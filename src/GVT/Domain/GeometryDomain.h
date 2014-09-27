//
// GeometryDomain.h
//
#ifndef GVT_GEOMETRY_DOMAIN_H
#define GVT_GEOMETRY_DOMAIN_H

#include <GVT/Data/primitives.h>
#include <GVT/Domain/Domain.h>

namespace GVT {

namespace Domain {

class GeometryDomain : public Domain {
 public:
  GVT::Data::Mesh* mesh;
  std::vector<GVT::Data::LightSource*> lights;
  std::string filename;

  GeometryDomain(std::string filename = "",
                 GVT::Math::AffineTransformMatrix<float> m =
                     GVT::Math::AffineTransformMatrix<float>(true))
      : Domain(m), mesh(NULL), filename(filename) {
    if (filename != "") {
      load();
      // free();
    }
  }

  virtual ~GeometryDomain() {}

  GeometryDomain(const GeometryDomain& other) : Domain(other) {
    mesh = other.mesh;
    lights = other.lights;
    boundingBox = other.boundingBox;
    filename = other.filename;
  }

  // virtual bool intersect(GVT::Data::ray&, vector<int>&);

  virtual bool hasGeometry() { return isLoaded; }

  virtual int size() { return 0; }

  virtual int sizeInBytes() { return 0; }

  virtual bool load();

  virtual void free();

  friend ostream& operator<<(ostream&, GeometryDomain const&);

  virtual void operator()() {}
};

}  // namespace Domain

}  // namespace GVT
#endif  // GVT_GEOMETRY_DOMAIN_H
