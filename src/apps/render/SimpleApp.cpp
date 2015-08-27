//
// Simple gravit application.
// Load some geometry and render it.
//
#include <vector>
#include <algorithm>
#include <set>
#include <gvt/core/mpi/Wrapper.h>
#include <gvt/core/Math.h>
#include <gvt/render/data/Dataset.h>
#include <gvt/render/data/Domains.h>
#include <gvt/render/Schedulers.h>
#include <gvt/render/adapter/manta/Wrapper.h>
//#include <gvt/render/adapter/optix/Wrapper.h>
#include <gvt/render/algorithm/Tracers.h>
#include <gvt/render/data/scene/gvtCamera.h>
#include <gvt/render/data/scene/Image.h>
#include <gvt/render/data/Primitives.h>
#include <gvt/render/Attributes.h>
#include <gvt/render/data/domain/reader/ObjReader.h>

#include <iostream>

#include <pthread.h>

using namespace std;
using namespace gvt::core::math;
using namespace gvt::core::mpi;
using namespace gvt::render::data::scene;
using namespace gvt::render::schedule;
using namespace gvt::render::data::primitives;
using namespace gvt::render::adapter::manta::data::domain;
using namespace gvt::render::algorithm;
//using namespace gvt::render::adapter::optix::data::domain;

std::vector<pthread_t> threads;
int imageWidth=1920;
int imageHeight=1080;
int tileWidth=32;
int tileHeight=32;
int numThreads=20;
int taskCounter=0;
int completedTasks=0;
int numTasks=(imageWidth*imageHeight/(tileWidth*tileHeight));
int thread_counter=0;
pthread_mutex_t thread_counter_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t task_counter_lock = PTHREAD_MUTEX_INITIALIZER;
pthread_mutex_t taskCompleted_counter_lock = PTHREAD_MUTEX_INITIALIZER;
gvt::render::algorithm::Tracer<ImageScheduler>* scheduler;

RenderContext rContext;

void* worker_thread(void* t)
{
  ThreadContext& tContext = *(ThreadContext*)t;
  
//  printf("thread %d/%d running\n",tContext.threadId,tContext.numThreads);
  
  WorkContext wContext;
  wContext.tileHeight=tileHeight;
  wContext.tileWidth=tileWidth;
  int counter =0;
  do
  {
    do
    {
      pthread_mutex_lock(&task_counter_lock);
      wContext.taskId  = taskCounter++;
      wContext.numTasks=numTasks;
      //    wContext.taskId=tContext.threadId+(counter++*tContext.numThreads);
      pthread_mutex_unlock(&task_counter_lock);
      if (wContext.taskId < numTasks)
      {
//              printf("thread %d/%d running task %d/%d\n",tContext.threadId, tContext.numThreads,wContext.taskId,wContext.numTasks);
        scheduler->render(rContext, tContext, wContext);
        pthread_mutex_lock(&taskCompleted_counter_lock);
        completedTasks++;
//              printf("thread %d/%d completed %d/%d\n",tContext.threadId, tContext.numThreads, completedTasks, numTasks);
        pthread_mutex_unlock(&taskCompleted_counter_lock);
      }
    } while (wContext.taskId < numTasks);
  } while (tContext.oneShot==false);
//  }
//  printf("thread %d finished\n", tContext.threadId);
  return 0;
}


int main(int argc, char** argv) {

////
////	our friend the cone.
////
//	Point4f points[7];
//        points[0] = Point4f(0.5,0.0,0.0,1.0);
//	points[1] = Point4f(-0.5,0.5,0.0,1.0);
//	points[2] = Point4f(-0.5,0.25,0.433013,1.0);
//	points[3] = Point4f(-0.5,-0.25,0.43013,1.0);
//	points[4] = Point4f(-0.5,-0.5,0.0,1.0);
//	points[5] = Point4f(-0.5,-0.25,-0.433013,1.0);
//	points[6] = Point4f(-0.5,0.25,-0.433013,1.0);
////
////	build a mesh object from cone geometry.
////
//	Mesh* objMesh = new Mesh(new Lambert(Vector4f(0.5,0.5,0.5,1.0)));
//	objMesh->addVertex(points[0]);
//	objMesh->addVertex(points[1]);
//	objMesh->addVertex(points[2]);
//	objMesh->addVertex(points[3]);
//	objMesh->addVertex(points[4]);
//	objMesh->addVertex(points[5]);
//	objMesh->addVertex(points[6]);
//
//	objMesh->addFace(1,2,3);
//	objMesh->addFace(1,3,4);
//	objMesh->addFace(1,4,5);
//	objMesh->addFace(1,5,6);
//	objMesh->addFace(1,6,7);
//	objMesh->addFace(1,7,2);
//
//	objMesh->generateNormals();

  gvt::render::data::domain::reader::ObjReader objReader("../data/geom/obj/bunny.obj");
  gvt::render::data::primitives::Mesh* objMesh = objReader.getMesh();

  objMesh->generateNormals();

	MPI_Init(&argc, &argv);
//
//	scene contains the geometry domain and the other elements like
//	camera, lights, etc.
//
	gvt::render::data::Dataset scene;

  //
  //	Add a point light at 1,1,1 and add it to the domain.
  //
  Vector4f   pos(1.0,1.0,1.0,0.0);
  Vector4f color(1.0,1.0,1.0,0.0);
  vector<gvt::render::data::scene::Light*> lightset;
  lightset.push_back(new gvt::render::data::scene::PointLight(pos,color));



    int numBunniesSqrt=1;
//
//	need a camera... using gvtCamera instead of default camera.... because I know how it works.
//
	gvtPerspectiveCamera*  mycamera = new gvtPerspectiveCamera();
	Point4f cameraposition(0,1.0,1,1.0);
	Point4f focus(0.0,0.5,0.0,1.0);
        if (numBunniesSqrt == 1)
        {
          cameraposition=Point4f(.1,.1,.2,1);
          focus = Point4f(0,.1,0,1);
        }
	float fov = 45.0 * M_PI/180.0;
	Vector4f up(0.0,1.0,0.0,0.0);
	mycamera->lookAt(cameraposition,focus,up);
	mycamera->setFOV(fov);
	mycamera->setFilmsize(imageWidth,imageHeight);
//
//	Create an object to hold the image and a pointer to the raw image data.
//
	Image* myimage = new Image(mycamera->getFilmSizeWidth(),mycamera->getFilmSizeHeight(),"bunnys");
	unsigned char *imagebuffer = myimage->GetBuffer();
//
//	Attributes class contains information used by tracer to generate image.
//	This will be replaced by the Context in the future.
//
	gvt::render::Attributes& rta =*(gvt::render::Attributes::instance());
	rta.view.width = mycamera->getFilmSizeWidth();
	rta.view.height = mycamera->getFilmSizeHeight();
	rta.schedule = gvt::render::Attributes::Image;
	rta.render_type = gvt::render::Attributes::Manta;
	rta.dataset = new gvt::render::data::Dataset();
	rta.do_lighting = true;

  //
  //	create a geometry domain and place the mesh inside
  //
  if (1)
  {
  for(int x=0;x<numBunniesSqrt;x++)
  {
    for(int y=0;y<numBunniesSqrt;y++)
    {
      gvt::core::math::AffineTransformMatrix<float> transform(true);
      transform = gvt::core::math::AffineTransformMatrix<float>::createTranslation(float(x)*0.2-(numBunniesSqrt-1)/9.0,float(y)*.2,0);
      gvt::render::data::domain::GeometryDomain* domain = new gvt::render::data::domain::GeometryDomain(objMesh,transform);
      scene.domainSet.push_back(domain );
      domain->setLights(lightset);
      rta.dataset->addDomain(new MantaDomain(domain));
    }
  }
  }
  //
  //add plane

  if (0)
  {
  float y = 0;
  float size =300;
  	Point4f points[6];
          points[0] = Point4f(size,y,-size,1.0);
  	points[1] = Point4f(-size,y,0.0,-size);
  	points[2] = Point4f(-size,y,size,1.0);
  	points[3] = Point4f(size,y,-size,1.0);
  	points[4] = Point4f(size,y,size,1.0);
  	points[5] = Point4f(-size,y,size,1.0);
//  	points[6] = Point4f(-300,y,-0.433013,1.0);
  //
  //	build a mesh object from cone geometry.
  //
  	Mesh* planeMesh = new Mesh(new Lambert(Vector4f(0.5,0.5,0.5,1.0)));
  	planeMesh->addVertex(points[0]);
  	planeMesh->addVertex(points[1]);
  	planeMesh->addVertex(points[2]);
  	planeMesh->addVertex(points[3]);
  	planeMesh->addVertex(points[4]);
  	planeMesh->addVertex(points[5]);

  planeMesh->addFace(1,2,3);
  planeMesh->addFace(4,5,6);
//  	objMesh->addFace(1,2,3);
//  	objMesh->addFace(1,3,4);
//  	objMesh->addFace(1,4,5);
//  	objMesh->addFace(1,5,6);
//  	objMesh->addFace(1,6,7);
//  	objMesh->addFace(1,7,2);

  	planeMesh->generateNormals();
  gvt::core::math::AffineTransformMatrix<float> transform(true);
//  transform = gvt::core::math::AffineTransformMatrix<float>::createTranslation(float(x)*0.2-.1,float(y)*.2-1,0);
  gvt::render::data::domain::GeometryDomain* domain = new gvt::render::data::domain::GeometryDomain(planeMesh);
  scene.domainSet.push_back(domain );
  domain->setLights(lightset);
  }
//  rta.dataset->addDomain(new MantaDomain(domain));
  
  rContext.camera = mycamera;
  rContext.image = myimage;
  scheduler = new gvt::render::algorithm::Tracer<ImageScheduler>(mycamera->rays,*myimage);
  threads.resize(numThreads);

  
  pthread_mutex_lock(&task_counter_lock);
  pthread_mutex_lock(&taskCompleted_counter_lock);
  
  ThreadContext* tContext = new ThreadContext();
  pthread_mutex_lock(&thread_counter_lock);
  tContext->threadId  = thread_counter++;
  pthread_mutex_unlock(&thread_counter_lock);
  tContext->oneShot=true;
  tContext->numThreads=numThreads;

  
  for(int i=1;i<threads.size();i++)
  {
    ThreadContext* tc = new ThreadContext();
    
    pthread_mutex_lock(&thread_counter_lock);
    tc->threadId  = thread_counter++;
    pthread_mutex_unlock(&thread_counter_lock);
    tc->numThreads=numThreads;
    
    pthread_create(&threads[i],NULL, &worker_thread, tc);
  }

//
//	Render it....
//	Hardwire the Manta adapter for this application.
//
  for (int i=0;i<5;i++)
  {
    boost::timer::auto_cpu_timer t("total render time %w\n");
    {
      boost::timer::auto_cpu_timer t("total ray time %w\n");
      //	mycamera.AllocateCameraRays();
      //	mycamera.generateRays();
    }
    {
      boost::timer::auto_cpu_timer t("tracer render time %w\n");
      taskCounter=0;
      completedTasks=0;
      pthread_mutex_unlock(&taskCompleted_counter_lock);
      pthread_mutex_unlock(&task_counter_lock);
      worker_thread(tContext); // run host thread as well
      bool done=false;
      //      while (completedTasks < numTasks) {}
      while (!done)
      {
        pthread_mutex_lock(&taskCompleted_counter_lock);
        done = (completedTasks >= numTasks);
        pthread_mutex_unlock(&taskCompleted_counter_lock);
      }
      
      printf("completed tasks, locking state\n");
      pthread_mutex_lock(&task_counter_lock);
      pthread_mutex_lock(&taskCompleted_counter_lock);
      printf("completed tasks, next loop\n");
    }
  }
  myimage->Write();
//  pthread_exit(NULL);
  return 0;
}
