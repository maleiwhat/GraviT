//
// MantaDomain.C
//

#include <common/utils.h>
#include <Backend/Optix/Domain/OptixDomain.h>
#include <Backend/Optix/Data/gvt_optix.h>

//TODO: Matt (if needed)

namespace GVT {
    namespace Domain {
        
        OptixDomain::OptixDomain(std::string filename) : GVT::Domain::GeometryDomain(filename) {
        }

        OptixDomain::OptixDomain(const OptixDomain& other) : GVT::Domain::GeometryDomain(other) {
        }

        OptixDomain::~OptixDomain() {  
        }
        
    };
};


