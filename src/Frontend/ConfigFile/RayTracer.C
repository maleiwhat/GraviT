//
//  RayTracer.C
//

#include "RayTracer.h"
#include <Model/Materials/Phong.h>
#include <Model/Readers/PlyReader.h>
#include <Interface/LightSet.h>
#include <Model/Lights/PointLight.h>

#include <GVT/Environment/Camera.h>
#include <GVT/Tracer/tracers.h>
#include <GVT/MPI/mpi_wrappers.h>
#include <Backend/Manta/gvtmanta.h>
//#include <assert.h>

#ifdef PARALLEL
#include <mpi.h>
#endif

#include <GVT/Scheduler/schedulers.h>


#include <boost/timer/timer.hpp>

void RayTracer::RenderImage(GVT::Env::Camera<C_PERSPECTIVE>& camera, Image& image) {

       boost::timer::auto_cpu_timer t("renderimage time: %ws\n");

    crays.resize((camera.getSPP()) * image.getWidth() * image.getHeight());
    {
       boost::timer::auto_cpu_timer t("make camera rays time: %ws\n");
       camera.MakeCameraRays(crays);
    }

    int render_type = GVT::Env::RayTracerAttributes::rta->render_type;

    //
    //  CARSON: I don't think we should be using arrays of pointers for ray storage unless absolutely necessary
    //
    {
       boost::timer::auto_cpu_timer t("convert rays time: %ws\n");
    size_t osize=rays.size();
    rays.resize(crays.size());
    for(size_t i=osize;i<rays.size();i++)
        rays[i] = new GVT::Data::ray();
    for(size_t i=0;i<rays.size();i++)
        *rays[i] = crays[i];
}

    switch (GVT::Env::RayTracerAttributes::rta->schedule) {
        case GVT::Env::RayTracerAttributes::Image:
            if (render_type == GVT::Env::RayTracerAttributes::Manta)
                GVT::Trace::Tracer<GVT::Domain::MantaDomain, MPICOMM, ImageSchedule>(rays, image)();
            else
                GVT::Trace::Tracer<GVT::Domain::VolumeDomain, MPICOMM, ImageSchedule>(rays, image)();
            break;
        case GVT::Env::RayTracerAttributes::Domain:
            if (render_type == GVT::Env::RayTracerAttributes::Manta)
                GVT::Trace::Tracer<GVT::Domain::MantaDomain, MPICOMM, DomainSchedule>(rays, image)();
            else
                GVT::Trace::Tracer< GVT::Domain::VolumeDomain, MPICOMM, DomainSchedule>(rays, image)();
            break;
        case GVT::Env::RayTracerAttributes::Greedy:
            if (render_type == GVT::Env::RayTracerAttributes::Manta)
                GVT::Trace::Tracer<GVT::Domain::MantaDomain, MPICOMM, HybridSchedule<GreedySchedule> >(rays, image)();
            else
                GVT::Trace::Tracer< GVT::Domain::VolumeDomain, MPICOMM, HybridSchedule<GreedySchedule> >(rays, image)();
            break;
        case GVT::Env::RayTracerAttributes::Spread:
            if (render_type == GVT::Env::RayTracerAttributes::Manta)
                GVT::Trace::Tracer<GVT::Domain::MantaDomain, MPICOMM, HybridSchedule<SpreadSchedule> >(rays, image)();
            else
                GVT::Trace::Tracer< GVT::Domain::VolumeDomain, MPICOMM, HybridSchedule<SpreadSchedule> >(rays, image)();
            break;
        case GVT::Env::RayTracerAttributes::RayWeightedSpread:
            if (render_type == GVT::Env::RayTracerAttributes::Manta)
                GVT::Trace::Tracer<GVT::Domain::MantaDomain, MPICOMM, HybridSchedule<RayWeightedSpreadSchedule> >(rays, image)();
            else
                GVT::Trace::Tracer< GVT::Domain::VolumeDomain, MPICOMM, HybridSchedule<RayWeightedSpreadSchedule> >(rays, image)();
            break;
        case GVT::Env::RayTracerAttributes::AdaptiveSend:
            if (render_type == GVT::Env::RayTracerAttributes::Manta)
                GVT::Trace::Tracer<GVT::Domain::MantaDomain, MPICOMM, HybridSchedule<AdaptiveSendSchedule> >(rays, image)();
            else
                GVT::Trace::Tracer< GVT::Domain::VolumeDomain, MPICOMM, HybridSchedule<AdaptiveSendSchedule> >(rays, image)();
            break;
        case GVT::Env::RayTracerAttributes::LoadOnce:
            if (render_type == GVT::Env::RayTracerAttributes::Manta)
                GVT::Trace::Tracer<GVT::Domain::MantaDomain, MPICOMM,  HybridSchedule<LoadOnceSchedule> >(rays, image)();
            else
                GVT::Trace::Tracer< GVT::Domain::VolumeDomain, MPICOMM, HybridSchedule<LoadOnceSchedule> >(rays, image)();
            break;
        case GVT::Env::RayTracerAttributes::LoadAnyOnce:
            if (render_type == GVT::Env::RayTracerAttributes::Manta)
                GVT::Trace::Tracer<GVT::Domain::MantaDomain, MPICOMM, HybridSchedule<LoadAnyOnceSchedule> >(rays, image)();
            else
                GVT::Trace::Tracer< GVT::Domain::VolumeDomain, MPICOMM, HybridSchedule<LoadAnyOnceSchedule> >(rays, image)();
            break;
        case GVT::Env::RayTracerAttributes::LoadAnother:
            if (render_type == GVT::Env::RayTracerAttributes::Manta)
                GVT::Trace::Tracer<GVT::Domain::MantaDomain, MPICOMM, HybridSchedule<LoadAnotherSchedule> >(rays, image)();
            else
                GVT::Trace::Tracer< GVT::Domain::VolumeDomain, MPICOMM, HybridSchedule<LoadAnotherSchedule> >(rays, image)();
            break;
        case GVT::Env::RayTracerAttributes::LoadMany:
            if (render_type == GVT::Env::RayTracerAttributes::Manta)
                GVT::Trace::Tracer<GVT::Domain::MantaDomain, MPICOMM, HybridSchedule<LoadManySchedule> >(rays, image)();
            else
                GVT::Trace::Tracer< GVT::Domain::VolumeDomain, MPICOMM, HybridSchedule<LoadManySchedule> >(rays, image)();
            break;
        default:
            cerr << "ERROR: unknown schedule '" << GVT::Env::RayTracerAttributes::rta->schedule << "'" << endl;
            return;
    }


};

#if !defined(M_PI)
#define M_PI 3.14159265358979323846
#endif




