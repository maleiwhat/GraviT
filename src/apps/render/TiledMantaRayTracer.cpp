//
//  TiledMantaRayTracer.cpp
//

#include "TiledMantaRayTracer.h"

#include <gvt/core/mpi/Wrapper.h>
// #include <gvt/render/adapter/manta/Wrapper.h>
// #include <gvt/render/algorithm/Tracers.h>
// #include <gvt/render/data/scene/Camera.h>
// #include <gvt/render/data/scene/Image.h>
// #include <gvt/render/Schedulers.h>
#include <gvt/render/Attributes.h>
#include <gvt/render/data/domain/GeometryDomain.h>
#include <gvt/render/adapter/manta/data/domain/MantaDomain.h>
#include <gvt/render/algorithm/TracerBase.h>
#include <gvt/render/algorithm/gvtState.h>
#include <gvt/render/algorithm/gvtDisplay.h>
#include <gvt/render/algorithm/gvtServer.h>
#include <gvt/render/algorithm/gvtWorker.h>


// Manta includes
#include <Interface/LightSet.h>
#include <Model/Lights/PointLight.h>
#include <Model/Materials/Phong.h>
#include <Model/Readers/PlyReader.h>
// end Manta includes

#include <boost/foreach.hpp>
#include <boost/timer/timer.hpp>

#ifdef PARALLEL
#include <mpi.h>
#endif

using namespace gvtapps::render;
using namespace gvt::core::mpi;
using namespace gvt::render::adapter::manta::data::domain;
using namespace gvt::render::data::domain;
using namespace gvt::render::data::scene;
// using namespace gvt::render::schedule;

using namespace cvt;

TiledMantaRayTracer::TiledMantaRayTracer(ConfigFileLoader& cl)
: scene(&cl.scene), m_width(512), m_height(512)
{
    scene->camera.SetCamera(rays,1.0);
	// uncomment this line to use gvtcamera
	//scene->GVTCamera.SetCamera(rays,1.0);
    
    gvt::render::Attributes& rta = *(gvt::render::Attributes::instance());
    gvt::render::Context cntxt = gvt::render::Context();   
    rta.dataset = new gvt::render::data::Dataset();
    
    
    BOOST_FOREACH(AbstractDomain* dom, scene->domainSet) 
    {
        GeometryDomain* d = (GeometryDomain*)dom;
        d->setLights(scene->lightSet);
        rta.dataset->addDomain(new MantaDomain(d));
    }

    // if (cl.accel_type != ConfigFileLoader::NoAccel)
    // {
    //     std::cout << "creating acceleration structure... ";
    //     if (cl.accel_type == ConfigFileLoader::BVH)
    //     {
    //         rta.accel_type = gvt::render::Attributes::BVH;
    //     }
    //     rta.dataset->makeAccel(rta);
    //     std::cout << "...done" << std::endl;
    // }
    
	// uncomment the following 2 lines to use gvtcamera 
    //rta.view.width = scene->GVTCamera.getFilmSizeWidth();
    //rta.view.height = scene->GVTCamera.getFilmSizeHeight();
    //
    // older camera setup. Comment out next two lines if using gvtcamera
    // rta.view.width = scene->camera.getFilmSizeWidth();
    // rta.view.height = scene->camera.getFilmSizeHeight();
    // 
    // the following rta variables never seem to be used commenting out
    rta.view.camera = scene->camera.getEye();
    rta.view.focus = scene->camera.getLook();
    //rta.view.up = scene->camera.up;
    
    //rta.sample_rate = 1.0f;
    //rta.sample_ratio = 1.0f;
    
    //rta.do_lighting = true;
    //rta.schedule = gvt::render::Attributes::Image;
    //rta.render_type = gvt::render::Attributes::Manta;
    
    //rta.datafile = "";
}

void TiledMantaRayTracer::RenderImage(std::string imagename = "mpitrace") 
{
    
 //    boost::timer::auto_cpu_timer t("Total render time: %t\n");

	// // comment out the following 3 lines to use gvt camera
    // Image image(scene->camera.getFilmSizeWidth(),scene->camera.getFilmSizeHeight(), imagename);
 //    // rays = scene->camera.MakeCameraRays();
 //    // gvt::render::algorithm::Tracer<DomainScheduler>(rays, image)();

 //    int rank,size;
 //    MPI_Comm_rank(MPI_COMM_WORLD  , &rank);
 //    MPI_Comm_size(MPI_COMM_WORLD, &size);
 //    bool multiProc = (size > 1);

 //    if (!multiProc || (multiProc && (rank == 0)))
 //    {
 //        gvtDisplay display;
 //        // display.domains = doms;
 //        display.width = g_width;
 //        display.height = g_height;
 //        // display.image = &image;
 //        display.Launch(argc, argv);
 //    }
 //    if (!multiProc || (multiProc && (rank == 1)))
 //    {
 //        gvtServer server;
 //        server.width = g_width;
 //        server.height = g_height;
 //        server.Launch(argc, argv);
 //    }
 //    if (!multiProc || (multiProc && (rank > 1)))
 //    {
 //        Worker worker;
 //        worker.Launch(argc, argv);
 //    }

// char buf[256];
    // string msg("hello");
  // if (argc > 1)
  //   msg = string(argv[1]);
    printf("client spawned\n");
    MPI_Comm parentcomm;
    int errcodes[1];
    // MPI_Init(&argc, &argv);
    int rank,size;
    MPI_Comm_rank(MPI_COMM_WORLD  , &rank);
    MPI_Comm_size(MPI_COMM_WORLD, &size);
  // boost::mpi::environment env;
  // boost::mpi::communicator world;
    MPI_Comm_get_parent(&parentcomm);
      // if (parentcomm == MPI_COMM_NULL)

    std::vector<StateDomain> doms;
  if(rank == 0) //gvtDisplay for now
  {

    glm::vec3 min(-50,-50,0);
      for(int i=0; i < 10;i++)
  {
    for (int j=0;j < 10;j++)
    {
      for(int k=0;k<1;k++)
      {
        StateDomain dom;
        dom.bound_min = glm::vec3(i*10+min[0],j*10+min[1],k*10+min[2]);
        dom.bound_max = glm::vec3(i*10+min[0]+9,j*10+min[1]+9,k*10+min[2]+9);
        dom.id = k*100+j*10+i;
        doms.push_back(dom);
      }
    }
  }
  // StateDomain dom;
  //   dom.bound_min = glm::vec3(0,0,0);
  //   dom.bound_max = glm::vec3(20,10,10);
  //   dom.id = 10;
  //   doms.push_back(dom);

    gvtDisplay display;
    display.domains = doms;
    display.width = m_width;
    display.height = m_height;
    display.imagename = imagename;
    display.Launch();
  }
  else if (rank == 1)
  {
    gvtServer server;
    server.width = m_width;
    server.height = m_height;
    server.Launch();
  } else //renderer
  {
    Worker worker;
    worker.Launch();
  }

// MPI_Finalize();
// return 0;

}
