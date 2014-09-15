#ifndef GVT_OPTIX_DOMAIN_H
#define GVT_OPTIX_DOMAIN_H

#include <string>

#include <optix_prime/optix_primepp.h>
#include <GVT/Domain/Domain.h>
#include <GVT/Domain/GeometryDomain.h>

namespace GVT {

namespace Data {
  class RayVector;
}  // namespace Data

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
  optix::prime::Context optix_context_;
  optix::prime::Model optix_model_;
};

}  // namespace Domain

}  // namespace GVT
#endif  // GVT_OPTIX_DOMAIN_H
