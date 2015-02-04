//
//  MPITrace.C
//

#include <config/config.h>
#ifdef GVT_MPE
#include "mpe.h"
#endif
#include <mpi.h>

#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <Frontend/ConfigFile/Dataset/Dataset.h>
#include <Frontend/ConfigFile/RayTracer.h>
#include <GVT/Data/primitives.h>
#include <GVT/Environment/RayTracerAttributes.h>
#include <GVT/Math/GVTMath.h>

#ifdef GVT_BE_MANTA
#include <Backend/Manta/gvtmanta.h>
#endif
#ifdef GVT_BE_OPTIX
#include <Backend/Optix/gvtoptix.h>
#endif
using namespace std;

int main(int argc, char** argv) {

#ifdef GVT_MPE
  int event1a, event1b, event2a, event2b, event3a, event3b;
  int event1, event2, event3;
#endif
  int rank = -1;
  MPI_Init(&argc, &argv);
  MPI_Comm_rank(MPI_COMM_WORLD, &rank);
  MPI_Barrier(MPI_COMM_WORLD);

  string filename, imagename;

  if (argc > 1)
    filename = argv[1];
  else
    filename = "./mpitrace.conf";

  if (argc > 2)
    imagename = argv[2];
  else
    imagename = "MPITrace";

#ifdef GVT_MPE
  // define some mpe events
  MPE_Log_get_state_eventIDs( &event1a, &event1b );
  MPE_Log_get_state_eventIDs( &event2a, &event2b );
  MPE_Log_get_state_eventIDs( &event3a, &event3b );
  MPE_Log_get_solo_eventID( &event1 );
  MPE_Log_get_solo_eventID( &event2 );
  MPE_Log_get_solo_eventID( &event3 );
  if(rank==0) {
	MPE_Describe_state( event1a, event1b, "Read geometry","red");
	MPE_Describe_state( event2a, event2b, "Render ","orange");
	MPE_Describe_state( event3a, event3b, "Composite ","green");
	MPE_Describe_event( event1, "Start Read", "white");
	MPE_Describe_event( event2, "Start Render", "purple");
	MPE_Describe_event( event3, "Start Composite", "navy");
  }
#endif
  fstream file;
  file.open(filename.c_str());

  if (!file.good()) {
    cerr << "ERROR: could not open file '" << filename << "'" << endl;
    return -1;
  }

  GVT::Env::RayTracerAttributes& rta =
      *(GVT::Env::RayTracerAttributes::instance());

  file >> rta;

  file.close();


  switch (rta.render_type) {
    case GVT::Env::RayTracerAttributes::Volume:
      GVT_DEBUG(DBG_ALWAYS, "Volume dataset");
	cerr << "VOLUME DATASET" << endl;
      rta.dataset =
          new GVT::Dataset::ConfigFileDataset<GVT::Domain::VolumeDomain>(
              rta.datafile);
      break;
    case GVT::Env::RayTracerAttributes::Surface:
      GVT_DEBUG(DBG_ALWAYS, "Geometry dataset");
	cerr << "GEOM DATASET" << endl;
      rta.dataset =
          new GVT::Dataset::ConfigFileDataset<GVT::Domain::GeometryDomain>(
              rta.datafile);
      break;
#ifdef GVT_BE_MANTA
    case GVT::Env::RayTracerAttributes::Manta:
      GVT_DEBUG(DBG_ALWAYS, "Using manta backend");
	cerr << "MANTA DATASET" << endl;
      rta.dataset = new GVT::Dataset::MantaDataset(rta.datafile);
      break;
#endif
#ifdef GVT_BE_OPTIX
    case GVT::Env::RayTracerAttributes::Optix:
      GVT_DEBUG(DBG_ALWAYS, "Using optix backend");
	cerr << "OPTIX DATASET" << endl;
      rta.dataset = new GVT::Dataset::OptixDataset(rta.datafile);
      break;
#endif
    default:
      GVT_DEBUG(DBG_ALWAYS, "GOT NUTHIN FOR A DATASET");
	cerr << "NUTTIN DATASET" << endl;
      break;
  }

#ifdef GVT_MPE
  MPE_Log_event(event1,0,NULL);
  MPE_Log_event(event1a,0,NULL);
#endif
  GVT_ASSERT(rta.LoadDataset(), "Unable to load dataset");
#ifdef GVT_MPE
  MPE_Log_event(event1b,0,NULL);
#endif
  std::cout << rta << std::endl;
  RayTracer rt;
  MPI_Barrier(MPI_COMM_WORLD);
  cout << "Rendering: rank=" << rank << endl;
#ifdef GVT_MPE
  MPE_Log_event(event2,0,NULL);
  MPE_Log_event(event2a,0,NULL);
#endif
  rt.RenderImage(imagename);
#ifdef GVT_MPE
  MPE_Log_event(event2b,0,NULL);
  MPE_Log_sync_clocks();
#endif
  cout << "Done rendering: rank=" << rank << endl;
  if (MPI::COMM_WORLD.Get_size() > 1) MPI_Finalize();
  cout << "Finalized: rank=" << rank << endl;

  return 0;
}
