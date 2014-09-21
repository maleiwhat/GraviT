//
//  RayTracer.C
//

#include "config/config.h"
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

void RayTracer::RenderImage(string imagename = "mpitrace") {
    Image image(GVT::Env::RayTracerAttributes::rta->view.width,
            GVT::Env::RayTracerAttributes::rta->view.height, imagename);
    GVT::Data::RayVector rays;

    GVT::Env::Camera<C_PERSPECTIVE> cam(
            rays, GVT::Env::RayTracerAttributes::rta->view,
            GVT::Env::RayTracerAttributes::rta->sample_rate);
    cam.MakeCameraRays();

    int render_type = GVT::Env::RayTracerAttributes::rta->render_type;

    switch (GVT::Env::RayTracerAttributes::rta->schedule) {
        case GVT::Env::RayTracerAttributes::Image:
            GVT::Trace::Tracer<MPICOMM, ImageSchedule>(
                    rays, image)();
            break;
        case GVT::Env::RayTracerAttributes::Domain:
            GVT::Trace::Tracer<MPICOMM, DomainSchedule>(
                    rays, image)();
            break;
        case GVT::Env::RayTracerAttributes::Greedy:
            GVT::Trace::Tracer<MPICOMM,
                    HybridSchedule<GreedySchedule> >(rays, image)();
            break;
        case GVT::Env::RayTracerAttributes::Spread:
            GVT::Trace::Tracer<MPICOMM,
                    HybridSchedule<SpreadSchedule> >(rays, image)();
            break;
        case GVT::Env::RayTracerAttributes::RayWeightedSpread:
            GVT::Trace::Tracer<MPICOMM,
                    HybridSchedule<RayWeightedSpreadSchedule> >(rays,
                    image)();
            break;
        case GVT::Env::RayTracerAttributes::AdaptiveSend:
            GVT::Trace::Tracer<MPICOMM,
                    HybridSchedule<AdaptiveSendSchedule> >(rays,
                    image)();
            break;
        case GVT::Env::RayTracerAttributes::LoadOnce:
            GVT::Trace::Tracer<MPICOMM,
                    HybridSchedule<LoadOnceSchedule> >(rays, image)();
            break;
            GVT::Trace::Tracer<MPICOMM,
                    HybridSchedule<LoadAnyOnceSchedule> >(rays, image)();
            break;
        case GVT::Env::RayTracerAttributes::LoadAnother:
            GVT::Trace::Tracer<MPICOMM,
                    HybridSchedule<LoadAnotherSchedule> >(rays, image)();
            break;
        case GVT::Env::RayTracerAttributes::LoadMany:
            GVT::Trace::Tracer<MPICOMM,
                    HybridSchedule<LoadManySchedule> >(rays, image)();
            break;
        default:
            cerr << "ERROR: unknown schedule '"
                    << GVT::Env::RayTracerAttributes::rta->schedule << "'" << endl;
            return;
    }

    int rank;
    MPI_Comm_rank(MPI_COMM_WORLD, &rank);
    if (rank == 0) {
        image.Write();
    }
};

#if !defined(M_PI)
#define M_PI 3.14159265358979323846
#endif

