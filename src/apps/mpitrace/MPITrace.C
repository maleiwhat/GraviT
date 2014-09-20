//
//  MPITrace.C
//


#include <config/config.h>
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

  MPI_Init(&argc, &argv);
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
      rta.dataset =
          new GVT::Dataset::ConfigFileDataset<GVT::Domain::VolumeDomain>(rta.datafile);
      break;
    case GVT::Env::RayTracerAttributes::Surface:
      GVT_DEBUG(DBG_ALWAYS, "Geometry dataset");
      rta.dataset =
          new GVT::Dataset::ConfigFileDataset<GVT::Domain::GeometryDomain>(rta.datafile);
      break;
#ifdef GVT_BE_MANTA
    case GVT::Env::RayTracerAttributes::Manta:
    GVT_DEBUG(DBG_ALWAYS,"Using manta backend");
    rta.dataset = new GVT::Dataset::MantaDataset(rta.datafile);
    break;
#endif
#ifdef GVT_BE_OPTIX
    case GVT::Env::RayTracerAttributes::Optix:
    GVT_DEBUG(DBG_ALWAYS,"Using optix backend");
        
    rta.dataset = new GVT::Dataset::OptixDataset(rta.datafile);
    break;
#endif
  }
  
  GVT_ASSERT(rta.LoadDataset(), "Unable to load dataset");
  std::cout << rta << std::endl;
  RayTracer rt;
  int rank = -1;
  MPI_Comm_rank (MPI_COMM_WORLD, &rank);
  MPI_Barrier(MPI_COMM_WORLD);
  cout << "Rendering: rank=" << rank << endl;
  rt.RenderImage(imagename);
  cout << "Done rendering: rank=" << rank << endl;
  if (MPI::COMM_WORLD.Get_size() > 1) MPI_Finalize();
  cout << "Finalized: rank=" << rank << endl;

  return 0;
}
