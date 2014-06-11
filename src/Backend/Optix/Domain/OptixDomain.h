

#ifndef GVT_OPTIX_DOMAIN_H
#define GVT_OPTIX_DOMAIN_H

#include <string>

#include <GVT/Domain/Domain.h>
#include <GVT/Domain/GeometryDomain.h>
#include <GVT/Data/primitives.h>
//#include <optix_prime/optix_primepp.h>
//#include <optixu/optixpp.h>
namespace GVT {
    namespace Domain {
        class OptixDomain : public GeometryDomain {
        public:
            OptixDomain(std::string filename = "");
            OptixDomain(const OptixDomain& other);
            virtual ~OptixDomain();

            void trace(GVT::Data::RayVector& rayList, GVT::Data::RayVector& moved_rays);
        };
    };
};


#endif // GVT_OPTIX_DOMAIN_H
