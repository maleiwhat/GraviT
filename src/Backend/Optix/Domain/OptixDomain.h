#ifndef GVT_OPTIX_DOMAIN_H
#define GVT_OPTIX_DOMAIN_H

#include <string>

#include <GVT/Domain/Domain.h>
#include <GVT/Domain/GeometryDomain.h>
#include <optix_prime/optix_primepp.h>

namespace GVT {

namespace Data {
class RayVector;
class ray;
}  // namespace Data

namespace Math {
class Vector4f;
}  // namespace Math

namespace Domain {

class OptixDomain : public GeometryDomain {
 public:
  OptixDomain();
  explicit OptixDomain(const std::string& filename);
  virtual ~OptixDomain();
  virtual bool load();
  void trace(Data::RayVector& rayList, Data::RayVector& moved_rays);
  void shade(RayVector& ray_list, std::vector<OptixHitFormat>& hits,
             RayVector& moved_rays);
  optix::prime::Context& optix_context() { return optix_context_; }
  optix::prime::Model& optix_model() {
    return optix_model_;
  };

 private:
  void traceRay(uint32_t triangle_id, float t, float u, float v,
                Data::ray& ray);
  Vector4f computeNormal(uint32_t triangle_id, float u, float v) const;
  void generateSecondaryRay(const Data::ray& ray, const Math::Vector4f& normal,
                            Data::RayVector& rays);
  void generateShadowRays(const Data::ray& ray, const Math::Vector4f& normal,
                          Data::RayVector& rays);
  optix::prime::Context optix_context_;
  optix::prime::Model optix_model_;
};

}  // namespace Domain

}  // namespace GVT
#endif  // GVT_OPTIX_DOMAIN_H
