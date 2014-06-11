//
// OptixDomain.C
//

#include <common/utils.h>
#include <Backend/Optix/Domain/OptixDomain.h>
#include <Backend/Optix/Data/gvt_optix.h>
#include <optix_prime/optix_primepp.h>
#include <optixu/optixpp.h>

//TODO: Matt (if needed)

namespace GVT {
    namespace Domain {
        
        OptixDomain::OptixDomain(std::string filename) : GVT::Domain::GeometryDomain(filename) {
        }

        OptixDomain::OptixDomain(const OptixDomain& other) : GVT::Domain::GeometryDomain(other) {
        }

        OptixDomain::~OptixDomain() {  
        }

        void OptixDomain::trace(GVT::Data::RayVector& rayList, GVT::Data::RayVector& moved_rays)
        {
          cout<<"Tracing Finally\n";
          optix::prime::Context context = optix::prime::Context::create(RTP_CONTEXT_TYPE_CUDA);
          optix::prime::Model model = context->createModel();
          GVT_DEBUG(DBG_ALWAYS, "processQueue<OptixDomain>: " << rayList.size());
          GVT_DEBUG(DBG_ALWAYS, "tracing geometry of domain " << domainID);

          optix::Ray * optixRays= new optix::Ray[rayList.size()];
          cout<<"Processing "<<rayList.size()<<" rays"<<endl;
          for(int i=0;i<rayList.size();i++)
            {
                optixRays[i].origin   =make_float3(rayList[i].origin[0],rayList[i].origin[1],rayList[i].origin[2]);
                optixRays[i].direction=make_float3(rayList[i].direction[0],rayList[i].direction[1],rayList[i].direction[2]);
                //optixRays[i].tmin     =rayList[i].tmin;
                //optixRays[i].tmax     =rayList[i].tmax;
            }
        }
        
    };
};


