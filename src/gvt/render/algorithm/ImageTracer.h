/*
 * ImageTracer.h
 *
 *  Created on: Nov 27, 2013
 *      Author: jbarbosa
 */

#ifndef GVT_RENDER_ALGORITHM_IMAGE_TRACER_H
#define GVT_RENDER_ALGORITHM_IMAGE_TRACER_H

#include <mpi.h>


#include <gvt/core/mpi/Wrapper.h>
#include <gvt/render/Schedulers.h>
#include <gvt/render/algorithm/TracerBase.h>

#include <boost/timer/timer.hpp>

#include <algorithm>

namespace gvt {
    namespace render {
        namespace algorithm {



          struct RenderContext
          {
            gvt::render::data::scene::Image* image;
            gvt::render::data::scene::gvtPerspectiveCamera* camera;
//            gvt::render::algorithm::Tracer<ImageScheduler>* scheduler;
          };

          struct WorkContext
          {
            WorkContext() { taskId=-1;numTasks=0;}
            int taskId;
            int numTasks;
            int tileHeight;
            int tileWidth;
          };

#define MAX_HITLISTDEPTH 12
          struct ThreadContext
          {
            ThreadContext()
            {
              oneShot=false;
              hitListOverflow=false;
            }
            //gvt::render::actor::RayVector rays;
            gvt::render::actor::RayPacket rays;
            int hitList[MAX_RAYPACKET][MAX_HITLISTDEPTH+1];
            bool hitListOverflow;
            int threadId;
            int numThreads;
            bool oneShot;
          };

            /// Tracer Image (ImageSchedule) based decomposition implementation

            template<> class Tracer<gvt::render::schedule::ImageScheduler> : public AbstractTrace
            {
            public:

                size_t rays_start, rays_end ;

                Tracer(gvt::render::actor::RayVector& rays, gvt::render::data::scene::Image& image)
                : AbstractTrace(rays, image)
                {
                    int ray_portion = rays.size() / mpi.world_size;
                    rays_start = mpi.rank * ray_portion;
                    rays_end = (mpi.rank + 1) == mpi.world_size ? rays.size() : (mpi.rank + 1) * ray_portion; // tack on any odd rays to last proc
                }

                virtual void FilterRaysLocally() {
                    if(mpi) {
                        gvt::render::actor::RayVector lrays;
                        lrays.assign(rays.begin() + rays_start,
                                     rays.begin() + rays_end);
                        rays.clear();
                        shuffleRays(lrays);
                    } else {
                        shuffleRays(rays);
                    }
                }

                virtual void operator()()
                {
                }
                virtual void render(RenderContext& rContext, ThreadContext& tContext, const WorkContext& wContext)
                {
                  //Carson trace
                  //
                  //
                  

                  //compute camera rays for threadContext rays
//                  threadContext.rays[i] = ... workContext.taskId/workContext.numTasks ...;
                  
//                  int buffer_width  = filmsize[0];
//                  int buffer_height = filmsize[1];
                  int tileWidth =wContext.tileWidth;
                  int tileHeight =wContext.tileHeight;
                  int raysCounter=0;
                  int numTilesX =rContext.image->width/wContext.tileWidth;
                  int tx = (wContext.taskId%(numTilesX))*tileWidth;
                  int ty = ((wContext.taskId)/(numTilesX))*tileHeight;
                  int maxRays =rContext.image->width*rContext.image->height;
                  
                  tileWidth = std::min(tileWidth, rContext.image->width-(tx));
                  tileHeight = std::min(tileHeight, rContext.image->height-(ty));
                  
                  int numRaysInTile= (tileWidth*tileHeight);
                  float aspectRatio = float(rContext.image->width)/float(rContext.image->height);
                  float fov = rContext.camera->field_of_view;
//                  printf("render\n");
                  while (raysCounter<numRaysInTile)
                  {
//                    printf("render packet\n");
//                    int i,j,idx;
                    float x,y;
                    int depth =rContext.camera->depth;
                    // these basis directions are scaled by the aspect ratio and
                    // the field of view.
                    gvt::core::math::Vector4f camera_vert_basis_vector  = gvt::core::math::Vector4f(0,1,0,0)*tan(fov*0.5);
                    gvt::core::math::Vector4f camera_horiz_basis_vector = gvt::core::math::Vector4f(1,0,0,0)*tan(fov*0.5)*aspectRatio;
                    gvt::core::math::Vector4f camera_normal_basis_vector = gvt::core::math::Vector4f(0,0,1,0);
                    gvt::core::math::Vector4f camera_space_ray_direction;
//                    for(j=0;j<wContext.tileHeight;j++)
//                    {
//                      for(i=0;i<wContext.tileWidth;i++)
//                      {
                    int idr=0;
                    for(int r=raysCounter;r<numRaysInTile;r++)
                    {
                        gvt::render::actor::Ray ray;
//                      printf("tile ratio %d %d: %d\n", rContext.image->height, wContext.tileHeight, rContext.image->height/wContext.tileHeight);
                        int ix = tx+r%tileWidth;
                        int iy = ty+r/tileWidth;
                        ray.id = ix + iy*rContext.image->width;
                        if (ray.id >= maxRays)
                          printf("ray id out of bounds ixiy: %d %d\n", ix,iy);
                        ray.w      = 1.0; // ray weight 1 for no subsamples. mod later
                        ray.origin = rContext.camera->eye_point;
                        ray.type   = gvt::render::actor::Ray::PRIMARY;
                        // calculate scale factors -1.0 < x,y < 1.0
                        x = 2.0*float(ix)/float(rContext.image->width - 1) - 1.0;
                        y = 2.0*float(iy)/float(rContext.image->height - 1) - 1.0;
                        // calculate ray direction in camera space;
                        camera_space_ray_direction = camera_normal_basis_vector + x*camera_horiz_basis_vector + y*camera_vert_basis_vector;
                        // transform ray to world coordinate space;
                        ray.setDirection(rContext.camera->cam2wrld*camera_space_ray_direction.normalize());
                        ray.depth = depth;
                        tContext.rays.setRay(idr, ray);
                        idr++;
                        raysCounter++;
                        if (idr==MAX_RAYPACKET)
                          break;
                    }
                    tContext.rays.resize(idr);
                    if (idr <= 0)
                      break;
//                    printf("trace doms\n");

                  //instead of shuffling onto ray queues, do one domain at a time.  Still stupid but better than n ray queus
                    
//                  while rays are still going

                     //intersect active rays, store in threadContext hitList
                    
                    for(int i=tContext.rays.begin();i!=tContext.rays.end();i++)
                    {
                      gvt::render::actor::isecDomList doms;
                                            gvt::render::Attributes::rta->dataset->intersect(tContext.rays.getRay(i), doms);
                      if (doms.size())
                      {
                        //                        printf("doms: ");
                        tContext.hitList[i][0]=0;
                        for(int j=0;j<doms.size();j++)
                        {
                          int hitSize = tContext.hitList[i][0];
                          if (hitSize < MAX_HITLISTDEPTH)
                          {
                            tContext.hitList[i][hitSize+1]=doms[j];
                            tContext.hitList[i][0]++;
                          }
                          else
                            tContext.hitListOverflow=true;  //TODO: Handle this                        }
                          //                          printf("%d ",doms[j]);
                          //                        printf("\n");
//                          tContext.rays.getRay(i).color.rgba[3]=1.f;
//                          tContext.rays.getRay(i).color.rgba[2]=1.f;
                        }
                      }
                    }

                     //walk hitlist, creating subpackets where needed
                       // trace subpacket with domain
                       // in case of overflow hitlist size, loop
                    
                    //Carson:  I'm following the in order traversal... but I don't think this makes sense.  I say just intersect all packet rays
                    // with active domain, why bother sorting them out?
                    //
                    gvt::render::data::domain::AbstractDomain* dom;
                    for(int i=0;i<MAX_HITLISTDEPTH;i++)
                    {
                      int domId=-1;
                      int start=tContext.rays.begin();
                      for(int j=tContext.rays.begin(); j!= tContext.rays.end();j++)
                      {
                        if (i >= tContext.hitList[j][0])
                          break;
                        int rd = tContext.hitList[j][i+1];
                        if (domId != -1 && rd != domId)
                        {
//                          gvt::render::actor::RayPacket subPacket(tContext.rays,start,j);
//                          dom = gvt::render::Attributes::rta->dataset->getDomain(domId);
//                          dom->load();
//                          dom->trace(subPacket);
                          
                          start=j+1;
                        }
                        domId = rd;
                      }
                      if (domId != -1 && start < tContext.rays.end())
                      {
                        gvt::render::actor::RayPacket subPacket(tContext.rays,start,tContext.rays.end());
                        dom = gvt::render::Attributes::rta->dataset->getDomain(domId);
                        dom->load();
                        dom->trace(subPacket);
                        for(int sr =subPacket.begin();sr< subPacket.end();sr++)
                          tContext.rays[start+sr] = subPacket[sr];
                      }
                    }

                  //set buffer values
                    gvt::render::data::scene::Image* image = rContext.image;
                    for(int i=tContext.rays.begin();i!=tContext.rays.end();i++)
                    {
//                      gvt::render::actor::Ray& r = tContext.rays.getRay(i);
//                    r.color.rgba[3]=1.f;
                      image->Add(tContext.rays.getId(i),tContext.rays.getColor(i).rgba);
//                      float c[] = {1,1,1,1};
//                      image->Add(tContext.rays.getId(i), c);
                    }
                  }

                }

                virtual void traceOld(RenderContext& context, ThreadContext& threadContext, WorkContext& workContext)
                {
                    GVT_DEBUG(DBG_ALWAYS,"Using Image schedule");
                    boost::timer::auto_cpu_timer t("tracer run %w\n");

                    long ray_counter = 0, domain_counter = 0;

                    // gvt::render::actor::RayVector local;
                    // local.assign(rays.begin()+rays_start, rays.begin()+ray_end);

                    {
                      boost::timer::auto_cpu_timer t("filter rays locally %w\n");
                      FilterRaysLocally();
                    }
                    {
                      boost::timer::auto_cpu_timer t2("tracer moved rays %w\n");

                    // buffer for color accumulation
                    gvt::render::actor::RayVector moved_rays;
                    int domTarget = -1, domTargetCount = 0;
                    // process domains until all rays are terminated
                    do
                    {
                      boost::timer::auto_cpu_timer t("tracer: domain loop %w\n");
                        // process domain with most rays queued
                        domTarget = -1;
                        domTargetCount = 0;

                        GVT_DEBUG(DBG_ALWAYS, "Selecting new domain");
                        for (std::map<int, gvt::render::actor::RayVector>::iterator q = this->queue.begin(); q != this->queue.end(); ++q)
                        {
                            if (q->second.size() > domTargetCount)
                            {
                                domTargetCount = q->second.size();
                                domTarget = q->first;
                            }
                        }
                        GVT_DEBUG(DBG_ALWAYS, "Selecting new domain");
                        //if (domTarget != -1) std::cout << "Domain " << domTarget << " size " << this->queue[domTarget].size() << std::endl;
                        // GVT_DEBUG_CODE(DBG_ALWAYS,if (DEBUG_RANK) std::cerr << mpi.rank << ": selected domain " << domTarget << " (" << domTargetCount << " rays)" << std::endl);
                        // GVT_DEBUG_CODE(DBG_ALWAYS,if (DEBUG_RANK) std::cerr << mpi.rank << ": currently processed " << ray_counter << " rays across " << domain_counter << " domains" << std::endl);

                        if (domTarget >= 0)
                        {
                                boost::timer::auto_cpu_timer t("Processing domTarget %w\n");

                            gvt::render::data::domain::AbstractDomain* dom;
                            GVT_DEBUG(DBG_ALWAYS, "Getting domain " << domTarget << std::endl);
                            {
                                boost::timer::auto_cpu_timer t("Loading dom %w\n");
                            dom = gvt::render::Attributes::rta->dataset->getDomain(domTarget);
                            dom->load();
                            }
                            GVT_DEBUG(DBG_ALWAYS, "dom: " << domTarget << std::endl);

                            // track domain loads
                            ++domain_counter;

                            GVT_DEBUG(DBG_ALWAYS, "Calling process queue");
                            //GVT::Backend::ProcessQueue<DomainType>(new GVT::Backend::adapt_param<DomainType>(this->queue, moved_rays, domTarget, dom, this->colorBuf, ray_counter, domain_counter))();
                            {
                                boost::timer::auto_cpu_timer t("Processing domain rays %w\n");
                                moved_rays.reserve(this->queue[domTarget].size()*10);
                                boost::timer::auto_cpu_timer t2("Tracing domain rays %w\n");
                                dom->trace(this->queue[domTarget], moved_rays);
                            }
                            GVT_DEBUG(DBG_ALWAYS, "Marching rays");
                            {
                              boost::timer::auto_cpu_timer t("tracer: shuffle dom rays %w\n");
                              shuffleRays(moved_rays,dom);
                            }
                            moved_rays.clear();
                        }
                    } while (domTarget != -1);
                    }
                    {
                      boost::timer::auto_cpu_timer t("tracer: gather framebuffers %w\n");
                    GVT_DEBUG(DBG_ALWAYS, "Gathering buffers");
                    this->gatherFramebuffers(this->rays.size());
                    }
                }
            };
        }
    }
}
#endif /* GVT_RENDER_ALGORITHM_IMAGE_TRACER_H */
