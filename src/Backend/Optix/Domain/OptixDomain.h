#ifndef GVT_OPTIX_DOMAIN_H
#define GVT_OPTIX_DOMAIN_H

#include <string>

#include <GVT/Data/primitives/gvt_ray.h>
#include <GVT/Domain/Domain.h>
#include <GVT/Domain/GeometryDomain.h>
#include <GVT/Math/Vector.h>
#include <optix_prime/optix_primepp.h>

namespace GVT {

namespace Domain {

class OptixDomain : public GeometryDomain {
 public:
  OptixDomain();
  explicit OptixDomain(const std::string& filename);
  virtual ~OptixDomain();
  virtual bool load();
  void trace(GVT::Data::RayVector& rayList, GVT::Data::RayVector& moved_rays);
  optix::prime::Context& optix_context() { return optix_context_; }
  optix::prime::Model& optix_model() {
    return optix_model_;
  };

 private:
  void traceRay(uint32_t triangle_id, float t, float u, float v,
                Data::ray& ray);
  GVT::Math::Vector4f computeNormal(uint32_t triangle_id, float u,
                                    float v) const;
  void generateSecondaryRay(const GVT::Data::ray& ray,
                            const GVT::Math::Vector4f& normal,
                            Data::RayVector& rays);
  void generateShadowRays(const GVT::Data::ray& ray,
                          const GVT::Math::Vector4f& normal,
                          GVT::Data::RayVector& rays);
  void traceRay(uint32_t triangle_id, float t, float u, float v, GVT::Data::ray& ray);
  optix::prime::Context optix_context_;
  optix::prime::Model optix_model_;
};

}  // namespace Domain

}  // namespace GVT
#endif  // GVT_OPTIX_DOMAIN_H
