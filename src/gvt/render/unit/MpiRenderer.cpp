/* =======================================================================================
   This file is released as part of GraviT - scalable, platform independent ray
   tracing
   tacc.github.io/GraviT

   Copyright 2013-2015 Texas Advanced Computing Center, The University of Texas
   at Austin
   All rights reserved.

   Licensed under the BSD 3-Clause License, (the "License"); you may not use
   this file
   except in compliance with the License.
   A copy of the License is included with this software in the file LICENSE.
   If your copy does not contain the License, you may obtain a copy of the
   License at:

       http://opensource.org/licenses/BSD-3-Clause

   Unless required by applicable law or agreed to in writing, software
   distributed under
   the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY
   KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under
   limitations under the License.

   GraviT is funded in part by the US National Science Foundation under awards
   ACI-1339863,
   ACI-1339881 and ACI-1339840
   =======================================================================================
   */

/** MpiRenderer.cpp
 *  A simple GraviT application that loads some geometry and renders it.
 */

#include "gvt/render/unit/MpiRenderer.h"

#include "gvt/core/Math.h"
#include "gvt/core/mpi/Wrapper.h"
#include "gvt/render/RenderContext.h"
#include "gvt/render/Schedulers.h"
#include "gvt/render/Types.h"
#include "gvt/render/data/Domains.h"
#include <algorithm>
#include <set>
#include <sys/stat.h>
#include <vector>

#ifdef GVT_RENDER_ADAPTER_EMBREE
#include "gvt/render/adapter/embree/Wrapper.h"
#endif

#ifdef GVT_RENDER_ADAPTER_MANTA
#include "gvt/render/adapter/manta/Wrapper.h"
#endif

#ifdef GVT_RENDER_ADAPTER_OPTIX
#include "gvt/render/adapter/optix/Wrapper.h"
#endif

#include "gvt/render/algorithm/Tracers.h"
#include "gvt/render/data/Primitives.h"
#include "gvt/render/data/scene/Image.h"
#include "gvt/render/data/scene/gvtCamera.h"
// #include "gvt/render/data/domain/reader/ObjReader.h"
#include "gvt/render/data/accel/BVH.h"

#include "gvt/core/mpi/Application.h"
#include <boost/range/algorithm.hpp>

#include "gvt/render/unit/DomainTracerProfiling.h"
#include "gvt/render/unit/ImageTileWork.h"
#include "gvt/render/unit/PixelWork.h"
#include "gvt/render/unit/RequestWork.h"
#include "gvt/render/unit/TileLoadBalancer.h"
#include "gvt/render/unit/TileWork.h"
#include "gvt/render/unit/Works.h"

#include "gvt/render/algorithm/DomainTracer.h"
#include "gvt/render/algorithm/ImageTracer.h"

#include <boost/timer/timer.hpp>
#include <iostream>
#include <mpi.h>

#include <tbb/task_scheduler_init.h>
#include <thread>

#include <math.h>
#include <stdio.h>

#include <queue>

#include "apps/render/TestScenes.h"
#include "gvt/render/unit/Voter.h"

#ifdef GVT_RENDER_ADAPTER_EMBREE
#include <gvt/render/adapter/embree/Wrapper.h>
#endif

#ifdef GVT_RENDER_ADAPTER_MANTA
#include <gvt/render/adapter/manta/Wrapper.h>
#endif

#ifdef GVT_RENDER_ADAPTER_OPTIX
#include <gvt/render/adapter/optix/Wrapper.h>
#endif

#if defined(GVT_RENDER_ADAPTER_OPTIX) && defined(GVT_RENDER_ADAPTER_EMBREE)
#include <gvt/render/adapter/heterogeneous/Wrapper.h>
#endif

#include <tbb/mutex.h>

// #define DEBUG_MPI_RENDERER
// #define DEBUG_RAYTX
// #define DEBUG_VOTER
#define PROFILE_RAY_COUNTS
#define ENABLE_MESSAGE

using namespace std;
using namespace gvt::render;
using namespace gvt::core;
using namespace gvt::core::mpi;
using namespace gvt::render::data::scene;
using namespace gvt::render::schedule;
using namespace gvt::render::data::primitives;
using namespace gvt::render::unit;
using namespace gvt::render::actor;

MpiRenderer::MpiRenderer(int *argc, char ***argv)
    : Application(argc, argv), camera(NULL), image(NULL), tileLoadBalancer(NULL), voter(NULL), adapter(NULL),
      acceleration(NULL), rayQMutex(NULL), colorBufMutex(NULL) {}

MpiRenderer::~MpiRenderer() {
  if (camera != NULL) delete camera;
  if (image != NULL) delete image;
  if (tileLoadBalancer != NULL) delete tileLoadBalancer;
  if (adapter != NULL) delete adapter;
  if (acceleration) delete acceleration;
  if (rayQMutex) delete[] rayQMutex;
  if (colorBufMutex) delete[] colorBufMutex;
  if (voter) delete voter;
}

void MpiRenderer::printUsage(const char *argv) {
  printf("Usage : %s [-h] [-i <infile>] [-a <adapter>] [-n <x y z>] [-p] [-s <scheduler>] [-W <image_width>] [-H "
         "<image_height>] [-N <num_frames>] [-t <num_tbb_threads>]\n",
         argv);
  printf("  -h, --help\n");
  printf("  -i, --infile <infile> (default: ../data/geom/bunny.obj for obj and ./EnzoPlyData/Enzo8 for ply)\n");
  printf("  -a, --adapter <embree | manta | optix> (default: embree)\n");
  printf("  -s, --scheduler <0-3> (default: 1)\n");
  printf("      0: AsyncImage (n/a), 1: AysncDomain, 2: SyncImage, 3:SyncDomain\n");
  printf("  -n, --num-instances <x, y, z> specify the number of instances in each direction (default: 1 1 1). "
         "effective only with obj.\n");
  printf("  -p, --ply use ply models\n");
  printf("  -W, --width <image_width> (default: 1280)\n");
  printf("  -H, --height <image_height> (default: 720)\n");
  printf("  -N, --num-frames <num_frames> (default: 1)\n");
  printf("  -t, --tbb <num_tbb_threads>\n");
  printf("      (default: # cores for sync. schedulers or # cores - 2 for async. schedulers)\n");
}

void MpiRenderer::parseCommandLine(int argc, char **argv) {
  options.numTbbThreads = -1;
  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
      printUsage(argv[0]);
      exit(1);
    } else if (strcmp(argv[i], "-i") == 0 || strcmp(argv[i], "--infile") == 0) {
      options.infile = argv[++i];
      struct stat buf;
      if (stat(options.infile.c_str(), &buf) != 0) {
        printf("error: file not found. %s\n", options.infile.c_str());
        exit(1);
      }
    } else if (strcmp(argv[i], "-a") == 0 || strcmp(argv[i], "--adapter") == 0) {
      ++i;
      if (strcmp(argv[i], "embree") == 0) {
        options.adapterType = gvt::render::adapter::Embree;
      } else if (strcmp(argv[i], "manta") == 0) {
        options.adapterType = gvt::render::adapter::Manta;
      } else if (strcmp(argv[i], "optix") == 0) {
        options.adapterType = gvt::render::adapter::Optix;
      } else {
        printf("error: %s not defined\n", argv[i]);
        exit(1);
      }
    } else if (strcmp(argv[i], "-s") == 0 || strcmp(argv[i], "--scheduler") == 0) {
      options.schedulerType = atoi(argv[++i]);
      if (options.schedulerType < 0 || options.schedulerType >= MpiRendererOptions::NumSchedulers) {
        printf("error: %s not defined\n", argv[i]);
        exit(1);
      }
    } else if (strcmp(argv[i], "-n") == 0 || strcmp(argv[i], "--num-instances") == 0) {
      options.instanceCountX = atoi(argv[++i]);
      options.instanceCountY = atoi(argv[++i]);
      options.instanceCountZ = atoi(argv[++i]);
    } else if (strcmp(argv[i], "-p") == 0 || strcmp(argv[i], "--ply") == 0) {
      options.ply = true;
    } else if (strcmp(argv[i], "-W") == 0 || strcmp(argv[i], "--width") == 0) {
      options.width = atoi(argv[++i]);
    } else if (strcmp(argv[i], "-H") == 0 || strcmp(argv[i], "--height") == 0) {
      options.height = atoi(argv[++i]);
    } else if (strcmp(argv[i], "-N") == 0 || strcmp(argv[i], "--num-frames") == 0) {
      options.numFrames = atoi(argv[++i]);
    } else if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--tbb") == 0) {
      options.numTbbThreads = atoi(argv[++i]);
    } else {
      printf("error: %s not defined\n", argv[i]);
      exit(1);
    }
  }
  if (options.numTbbThreads <= 0) {
    if (options.schedulerType == 2 || options.schedulerType == 3) {
      options.numTbbThreads = MAX(1, std::thread::hardware_concurrency());
    } else {
      options.numTbbThreads = MAX(1, std::thread::hardware_concurrency() - 2);
    }
  }
  if (options.infile.empty()) {
    if (options.ply) {
      options.infile = std::string("./EnzoPlyData/Enzo8/");
    } else {
      options.infile = std::string("../data/geom/bunny.obj");
    }
  }
}

void MpiRenderer::createDatabase() {
  // create scene
  apps::render::TestScenes scene(options);
  if (options.ply) {
    scene.makePlyDatabase();
  } else {
    scene.makeObjDatabase();
  }
  // create camera
  auto renderContext = gvt::render::RenderContext::instance();
  if (renderContext == NULL) {
    std::cout << "Something went wrong initializing the context" << std::endl;
    exit(1);
  }
  gvt::core::DBNodeH root = renderContext->getRootNode();
  gvt::core::DBNodeH node = renderContext->createNodeFromType("Camera", "cam", root.UUID());
  glm::vec3 eye(0.0, 0.5, 1.2);
  glm::vec3 focus(0.0, 0.0, 0.0);
  glm::vec3 upVector(0.0, 1.0, 0.0);
  float fov = (45.0 * M_PI / 180.0);
  if (options.ply) {
    eye = glm::vec3(512.0, 512.0, 4096.0);
    focus = glm::vec3(512.0, 512.0, 0.0);
    upVector = glm::vec3(0.0, 1.0, 0.0);
    fov = (float)(25.0 * M_PI / 180.0);
  }
  node["eyePoint"] = eye;
  node["focus"] = focus;
  node["upVector"] = upVector;
  node["fov"] = fov;
  camera = new gvtPerspectiveCamera();
  camera->lookAt(eye, focus, upVector);
  camera->setFOV(fov);
  camera->setFilmsize(options.width, options.height);
}

void MpiRenderer::initInstanceRankMap() {
  gvt::core::Vector<gvt::core::DBNodeH> dataNodes = root["Data"].getChildren();
#ifdef DEBUG_MPI_RENDERER
  std::cout << "instance node size: " << instancenodes.size() << "\ndata node size: " << dataNodes.size() << "\n";
#endif
  // create a map of instances to mpi rank
  for (size_t i = 0; i < instancenodes.size(); i++) {
    gvt::core::DBNodeH meshNode = instancenodes[i]["meshRef"].deRef();

    size_t dataIdx = -1;
    for (size_t d = 0; d < dataNodes.size(); d++) {
      if (dataNodes[d].UUID() == meshNode.UUID()) {
        dataIdx = d;
        break;
      }
    }

    // NOTE: mpi-data(domain) assignment strategy
    size_t mpiNode = dataIdx % GetSize();

    GVT_DEBUG(DBG_ALWAYS, "[" << GetRank() << "] domain scheduler: instId: " << i << ", dataIdx: " << dataIdx
                              << ", target mpi node: " << mpiNode << ", world size: " << GetSize());

    GVT_ASSERT(dataIdx != -1, "domain scheduler: could not find data node");
    mpiInstanceMap[i] = mpiNode;
  }
}

void MpiRenderer::setupCommon() {
  tbb::task_scheduler_init init(options.numTbbThreads);
  auto renderContext = gvt::render::RenderContext::instance();
  if (renderContext == NULL) {
    std::cout << "Something went wrong initializing the context" << std::endl;
    exit(1);
  }
  root = renderContext->getRootNode();
  instancenodes = root["Instances"].getChildren();
  GVT_DEBUG(DBG_ALWAYS, "num instances: " << instancenodes.size());
  imageWidth = root["Film"]["width"].value().toInteger();
  imageHeight = root["Film"]["height"].value().toInteger();
  colorBuf.resize(imageWidth * imageHeight);
  acceleration = new gvt::render::data::accel::BVH(instancenodes);
  rayQMutex = new tbb::mutex[instancenodes.size()];
  colorBufMutex = new tbb::mutex[imageWidth];
  numRanks = GetSize();
  myRank = GetRank();

  for (int i = 0; i < instancenodes.size(); i++) {
    meshRef[i] =
        (gvt::render::data::primitives::Mesh *)instancenodes[i]["meshRef"].deRef()["ptr"].value().toULongLong();
    instM[i] = (glm::mat4 *)instancenodes[i]["mat"].value().toULongLong();
    instMinv[i] = (glm::mat4 *)instancenodes[i]["matInv"].value().toULongLong();
    instMinvN[i] = (glm::mat3 *)instancenodes[i]["normi"].value().toULongLong();
  }

  auto lightNodes = root["Lights"].getChildren();
  lights.reserve(2);
  for (auto lightNode : lightNodes) {
    auto color = lightNode["color"].value().tovec3();

    if (lightNode.name() == std::string("PointLight")) {
      auto pos = lightNode["position"].value().tovec3();
      lights.push_back(new gvt::render::data::scene::PointLight(pos, color));
    } else if (lightNode.name() == std::string("AmbientLight")) {
      lights.push_back(new gvt::render::data::scene::AmbientLight(color));
    } else if (lightNode.name() == std::string("AreaLight")) {
      auto pos = lightNode["position"].value().tovec3();
      auto normal = lightNode["normal"].value().tovec3();
      auto width = lightNode["width"].value().toFloat();
      auto height = lightNode["height"].value().toFloat();
      lights.push_back(new gvt::render::data::scene::AreaLight(pos, color, normal, width, height));
    }
  }

  gatherTimesStart = false;
  pthread_mutex_init(&gatherTimesStartMutex, NULL);
  pthread_cond_init(&gatherTimesStartCond, NULL);

  gatherTimesDone = false;
  pthread_mutex_init(&gatherTimesDoneMutex, NULL);
  pthread_cond_init(&gatherTimesDoneCond, NULL);
}

void MpiRenderer::setupAsyncImage() {
  setupCommon();
  if (myRank == rank::Server) {
    serverReady = false;
    pthread_mutex_init(&serverReadyLock, NULL);
    pthread_cond_init(&serverReadyCond, NULL);
  }
}

void MpiRenderer::setupAsyncDomain() {
  setupCommon();
  initInstanceRankMap();

  pthread_mutex_init(&rayTransferBufferLock, NULL);
  pthread_mutex_init(&rayTransferMutex, NULL);

  imageReady = false;
  pthread_mutex_init(&imageReadyLock, NULL);
  pthread_cond_init(&imageReadyCond, NULL);

  if (numRanks > 1) {
    voter = new Voter(numRanks, myRank, &rayQ);
  }
}

void MpiRenderer::setupSyncImage() { setupCommon(); }

void MpiRenderer::setupSyncDomain() { setupCommon(); }

void MpiRenderer::freeRender() {
#ifdef DEBUG_MPI_RENDERER
  printf("Rank %d: MpiRenderer::freeRender. cleaning up.\n", GetRank());
#endif
  delete acceleration;
  delete[] rayQMutex;
  delete[] colorBufMutex;
  if (voter) delete voter;
}

void MpiRenderer::render() {
  switch (options.schedulerType) {
  // case MpiRendererOptions::AsyncImage: {
  //   renderAsyncImage();
  // } break;
  case MpiRendererOptions::AsyncDomain: {
    renderAsyncDomain();
  } break;
  case MpiRendererOptions::SyncImage: {
    renderSyncImage();
  } break;
  case MpiRendererOptions::SyncDomain: {
    renderSyncDomain();
  } break;
  default: {
    printf("error: unknown scheduler type %d\n", options.schedulerType);
    exit(1);
  } break;
  }
}

// void MpiRenderer::renderAsyncImage() {
//   setupAsyncImage();
//   GVT_ASSERT(options.numFrames == 1, "multiple frames not supported yet for image scheduler");
//   if (myRank == 0) printf("[async mpi] starting image scheduler using %d processes\n", GetSize());
//
//   RequestWork::Register();
//   TileWork::Register();
//   ImageTileWork::Register();
//   PixelWork::Register();
//
//   Application::Start();
//   if (myRank == rank::Server) {
//     initServer();
//   }
//   RequestWork request;
//   request.setSourceRank(GetRank());
//   request.Send(rank::Server);
//   Application::Wait();
// }

void MpiRenderer::renderAsyncDomain() {
  setupAsyncDomain();
  if (myRank == 0) printf("[async mpi] starting domain scheduler using %d processes\n", GetSize());

  RayTransferWork::Register();
  VoteWork::Register();
  PixelGatherWork::Register();
  TimeGatherWork::Register();
  Application::Start();

  image = new Image(imageWidth, imageHeight, "image");

  Timer t_total;
  Timer t_wait_composite;

  for (int i = 0; i < options.numFrames; ++i) {
#ifdef ENABLE_MESSAGE
    printf("[async mpi] Rank %d: frame %d start\n", myRank, i);
#endif
    runDomainTracer();

    t_wait_composite.start();
    pthread_mutex_lock(&imageReadyLock);

    while (!imageReady) pthread_cond_wait(&imageReadyCond, &imageReadyLock);
    imageReady = false;
    pthread_mutex_unlock(&imageReadyLock);

    t_wait_composite.stop();
    profiler.update(Profiler::WaitComposite, t_wait_composite.getElapsed());

    if (numRanks > 1) voter->reset();
  }

  t_total.stop();
  profiler.update(Profiler::Total, t_total.getElapsed());

  pthread_mutex_lock(&gatherTimesStartMutex);
  gatherTimesStart = true;
  pthread_cond_signal(&gatherTimesStartCond);
  pthread_mutex_unlock(&gatherTimesStartMutex);

  if (myRank == 0) {
    TimeGatherWork timeGather;
    timeGather.Broadcast(true, true);
  }

  pthread_mutex_lock(&gatherTimesDoneMutex);
  while (!gatherTimesDone) {
    pthread_cond_wait(&gatherTimesDoneCond, &gatherTimesDoneMutex);
  }
  gatherTimesDone = false;
  pthread_mutex_unlock(&gatherTimesDoneMutex);

  Application::Kill();
}

void MpiRenderer::renderSyncImage() {
  setupSyncImage();
  GVT_ASSERT(numRanks == 1, "multiple nodes not yet supported for the image scheduler");
  image = new Image(imageWidth, imageHeight, "image");

  if (myRank == 0) printf("[sync mpi] starting image scheduler without the mpi layer using %d processes\n", GetSize());

  gvt::render::algorithm::Tracer<ImageScheduler> tracer(camera->rays, *image);

  for (int i = 0; i < options.numFrames; ++i) {
    printf("[sync mpi] Rank %d: frame %d start\n", myRank, i);
    camera->AllocateCameraRays();
    camera->generateRays();
    image->clear();
    tracer();
    image->Write();
  }

  Quit::Register();
  Start();
  if (myRank == 0) {
    Quit quit;
    quit.Broadcast(true, true);
  }
}

void MpiRenderer::renderSyncDomain() {
  setupSyncDomain();
  image = new Image(imageWidth, imageHeight, "image");

  camera->AllocateCameraRays();
  camera->generateRays();

  Timer t_total;

  if (myRank == 0) printf("[sync mpi] starting domain scheduler without the mpi layer using %d processes\n", GetSize());
  gvt::render::algorithm::Tracer<gvt::render::schedule::DomainSchedulerProfiling> tracer(camera->rays, *image,
                                                                                         profiler);
  for (int i = 0; i < options.numFrames; ++i) {
#ifdef ENABLE_MESSAGE
    printf("[sync mpi] Rank %d: frame %d start\n", myRank, i);
#endif
    Timer t_primary;
    camera->AllocateCameraRays();
    camera->generateRays();
    t_primary.stop();
    profiler.update(Profiler::GenPrimaryRays, t_primary.getElapsed());

    image->clear();
    tracer();
    if (myRank == 0) image->Write();
   //  printf("[sync mpi] Rank %d: frame %d done\n", myRank, i);
  }

  t_total.stop();
  profiler.update(Profiler::Total, t_total.getElapsed());

  if (myRank == 0) {
    profiler.gtimes.resize(numRanks * Profiler::NumTimers);
#ifdef PROFILE_RAY_COUNTS
    profiler.grays.resize(numRanks);
#endif
  }

  MPI_Gather(static_cast<const void *>(&profiler.times[0]), Profiler::NumTimers, MPI_DOUBLE,
             static_cast<void *>(&profiler.gtimes[0]), Profiler::NumTimers, MPI_DOUBLE, 0, MPI_COMM_WORLD);

#ifdef PROFILE_RAY_COUNTS
  MPI_Gather(static_cast<const void *>(&profiler.rays), sizeof(Profiler::RayCounts), MPI_BYTE,
             static_cast<void *>(&profiler.grays[0]), sizeof(Profiler::RayCounts), MPI_BYTE, 0, MPI_COMM_WORLD);
#endif

  if (myRank == 0) {
    profiler.print(options.numFrames, numRanks);
  }

  Quit::Register();
  Start();
  if (myRank == 0) {
    Quit quit;
    quit.Broadcast(true, true);
  }
}

// void MpiRenderer::initServer() {
//   // TODO (hpark): For now, equally divide the image
//   // try coarser/finer granularity later
//   int schedType = root["Schedule"]["type"].value().toInteger();
//   // int numRanks = GetSize();
//   int numWorkers = numRanks;
//   int granularity = numRanks;
//
//   tileLoadBalancer = new TileLoadBalancer(schedType, imageWidth, imageHeight, granularity, numWorkers);
//   image = new Image(imageWidth, imageHeight, "image");
//   pendingPixelCount = imageWidth * imageHeight;
//
//   pthread_mutex_lock(&serverReadyLock);
//   serverReady = true;
//   pthread_cond_signal(&serverReadyCond);
//   pthread_mutex_unlock(&serverReadyLock);
// }

void MpiRenderer::aggregatePixel(int pixelId, const glm::vec3 &addend) { colorBuf[pixelId] += addend; }

bool MpiRenderer::transferRays() {

  Timer t_send;
  Timer t_receive;
  Timer t_vote;

  bool done;
  if (numRanks > 1) {

    if (voter->isCommunicationAllowed()) { // TODO: potential improvement
      t_send.start();
      sendRays();
      t_send.stop();
      profiler.update(Profiler::Send, t_send.getElapsed());

      t_receive.start();
      receiveRays();
      t_receive.stop();
      profiler.update(Profiler::Receive, t_receive.getElapsed());
    }

    t_vote.start();
    done = voter->updateState();
#ifdef DEBUG_VOTER
    if (myRank == 0) printf("rank %d: voter state %d\n", myRank, voter->state);
#endif
    t_vote.stop();
    profiler.update(Profiler::Vote, t_vote.getElapsed());

  } else {
    done = !hasWork();
  }

  assert(!done || (done && !hasWork()));

  return done;
}

bool MpiRenderer::hasWork() const {
  int not_done = 0;
  for (auto &q : rayQ) not_done += q.second.size();
  return (not_done > 0);
}

void MpiRenderer::sendRays() {
#ifdef PROFILE_RAY_COUNTS
  uint64_t rayCount = 0;
#endif
  for (auto &q : rayQ) {
    int instance = q.first;
    RayVector &rays = q.second;
    int ownerRank = mpiInstanceMap[instance];
    size_t numRaysToSend = rays.size();
    if (ownerRank != myRank && numRaysToSend > 0) {
      voter->addNumPendingRays(numRaysToSend);
      RayTransferWork work;
      work.setup(RayTransferWork::Request, myRank, instance, &rays);
      work.Send(ownerRank);
      rays.clear();
#ifdef PROFILE_RAY_COUNTS
      rayCount += numRaysToSend;
#endif
#ifdef DEBUG_RAYTX
      printf("rank %d: sent %lu rays instance %d to rank %d\n", myRank, numRaysToSend, instance, ownerRank);
#endif
    }
  }
#ifdef PROFILE_RAY_COUNTS
  profiler.addRayCountSend(rayCount);
#endif
}

void MpiRenderer::receiveRays() {
#ifdef PROFILE_RAY_COUNTS
  uint64_t rayCount = 0;
#endif
  pthread_mutex_lock(&rayTransferBufferLock);
  for (size_t i = 0; i < rayTransferBuffer.size(); ++i) {
    RayTransferWork *raytx = rayTransferBuffer[i];
    raytx->copyIncomingRays(&rayQ);

    RayTransferWork grant;
    grant.setup(RayTransferWork::Grant, myRank, raytx->getNumRays());
    grant.Send(raytx->getSenderRank());
#ifdef PROFILE_RAY_COUNTS
    rayCount += raytx->getNumRays();
#endif
#ifdef DEBUG_RAYTX
    printf("rank %d: recved %d rays instance %d \n", myRank, raytx->getNumRays(), raytx->getInstanceId());
#endif
    delete raytx;
  }
  rayTransferBuffer.clear(); // TODO: avoid this
  pthread_mutex_unlock(&rayTransferBufferLock);
#ifdef PROFILE_RAY_COUNTS
  profiler.addRayCountRecv(rayCount);
#endif
}

void MpiRenderer::bufferRayTransferWork(RayTransferWork *work) {
  pthread_mutex_lock(&rayTransferBufferLock);
  rayTransferBuffer.push_back(work); // TODO: avoid resizing
  pthread_mutex_unlock(&rayTransferBufferLock);
}

void MpiRenderer::bufferVoteWork(VoteWork *work) { voter->bufferVoteWork(work); }
void MpiRenderer::applyRayTransferResult(int numRays) { voter->subtractNumPendingRays(numRays); }

void MpiRenderer::copyIncomingRays(int instanceId, const gvt::render::actor::RayVector *incomingRays) {
  pthread_mutex_lock(&rayTransferMutex);
  if (rayQ.find(instanceId) != rayQ.end()) {
    rayQ[instanceId].insert(rayQ[instanceId].end(), incomingRays->begin(), incomingRays->end());
  } else {
    rayQ[instanceId] = *incomingRays;
  }
  pthread_mutex_unlock(&rayTransferMutex);
}

void MpiRenderer::runDomainTracer() {
  Timer t_primary;
  // RayVector rays;
  // generatePrimaryRays(rays);
  camera->AllocateCameraRays();
  camera->generateRays();

  t_primary.stop();
  profiler.update(Profiler::GenPrimaryRays, t_primary.getElapsed());

  domainTracer(camera->rays);
}

void MpiRenderer::generatePrimaryRays(RayVector &rays) {
  const int imageWidth = options.width;
  const int imageHeight = options.height;
  const int tileW = options.width;
  const int tileH = options.height;
  const int startX = 0;
  const int startY = 0;

  glm::vec3 eye = root["Camera"]["eyePoint"].value().tovec3();
  float fov = root["Camera"]["fov"].value().toFloat();
  const glm::mat4 &cam2wrld = camera->getCameraToWorld();

  rays.resize(tileW * tileH);

  // Generate rays direction in camera space and transform to world space.
  int i, j;
  int tilePixelId, imagePixelId;
  float aspectRatio = float(imageWidth) / float(imageHeight);
  float x, y;
  // these basis directions are scaled by the aspect ratio and
  // the field of view.
  // glm::vec3 camera_horiz_basis_vector = glm::vec3(1, 0, 0) * tan(fov * 0.5) * aspectRatio;
  // glm::vec3 camera_vert_basis_vector = glm::vec3(0, 1, 0) * tan(fov * 0.5);
  // glm::vec3 camera_normal_basis_vector = glm::vec3(0, 0, 1);
  const glm::vec3 z(cam2wrld[0][2], cam2wrld[1][2], cam2wrld[2][2]);

  for (j = startY; j < startY + tileH; j++) {
    for (i = startX; i < startX + tileW; i++) {
      // select a ray and load it up
      tilePixelId = (j - startY) * tileW + (i - startX);
      imagePixelId = j * imageWidth + i;
      Ray &ray = rays[tilePixelId];
      ray.id = imagePixelId;
      ray.w = 1.0; // ray weight 1 for no subsamples. mod later
      ray.origin = eye;
      ray.type = Ray::PRIMARY;
      // calculate scale factors -1.0 < x,y < 1.0
      x = 2.0 * float(i) / float(imageWidth - 1) - 1.0;
      y = 2.0 * float(j) / float(imageHeight - 1) - 1.0;
      // calculate ray direction in camera space;
      glm::vec3 camera_space_ray_direction;
      camera_space_ray_direction[0] = cam2wrld[0][0] * x + cam2wrld[0][1] * y + z[0];
      camera_space_ray_direction[1] = cam2wrld[1][0] * x + cam2wrld[1][1] * y + z[1];
      camera_space_ray_direction[2] = cam2wrld[2][0] * x + cam2wrld[2][1] * y + z[2];
      // camera_space_ray_direction =
      //     camera_normal_basis_vector + x * camera_horiz_basis_vector + y * camera_vert_basis_vector;
      // transform ray to world coordinate space;
      // ray.setDirection(cameraToWorld * glm::normalize(camera_space_ray_direction));
      ray.direction = glm::normalize(camera_space_ray_direction);
      ray.depth = 0;
    }
  }
}

void MpiRenderer::filterRaysLocally(RayVector &rays) {
  shuffleDropRays(rays);
  // auto nullNode = gvt::core::DBNodeH(); // temporary workaround until shuffleRays is fully replaced
  // GVT_DEBUG(DBG_ALWAYS, "image scheduler: filter locally non mpi: " << rays.size());
  // shuffleRays(rays, nullNode);
}

void MpiRenderer::shuffleDropRays(RayVector &rays) {
  const size_t chunksize = MAX(2, rays.size() / (std::thread::hardware_concurrency() * 4));
  static gvt::render::data::accel::BVH &acc = *dynamic_cast<gvt::render::data::accel::BVH *>(acceleration);
  static tbb::simple_partitioner ap;
  tbb::parallel_for(tbb::blocked_range<gvt::render::actor::RayVector::iterator>(rays.begin(), rays.end(), chunksize),
                    [&](tbb::blocked_range<gvt::render::actor::RayVector::iterator> raysit) {
                      std::vector<gvt::render::data::accel::BVH::hit> hits =
                          acc.intersect<GVT_SIMD_WIDTH>(raysit.begin(), raysit.end(), -1);
                      std::map<int, gvt::render::actor::RayVector> local_queue;
                      for (size_t i = 0; i < hits.size(); i++) {
                        gvt::render::actor::Ray &r = *(raysit.begin() + i);
                        if (hits[i].next != -1) {
                          r.origin = r.origin + r.direction * (hits[i].t * 0.8f);
                          const bool inRank = mpiInstanceMap[hits[i].next] == myRank;
                          if (inRank) local_queue[hits[i].next].push_back(r);
                        }
                      }
                      for (auto &q : local_queue) {
                        rayQMutex[q.first].lock();
                        rayQ[q.first].insert(rayQ[q.first].end(), std::make_move_iterator(local_queue[q.first].begin()),
                                             std::make_move_iterator(local_queue[q.first].end()));
                        rayQMutex[q.first].unlock();
                      }
                    },
                    ap);

  rays.clear();
}

void MpiRenderer::domainTracer(RayVector &rays) {
  Timer t_filter;
  Timer t_schedule;
  Timer t_adapter;
  Timer t_trace;
  Timer t_shuffle;
  Timer t_composite;

  GVT_DEBUG(DBG_ALWAYS, "domain scheduler: starting, num rays: " << rays.size());
  gvt::core::DBNodeH root = gvt::render::RenderContext::instance()->getRootNode();

  clearBuffer();
  int adapterType = root["Schedule"]["adapter"].value().toInteger();

  long domain_counter = 0;

  // FindNeighbors();

  // sort rays into queues
  // note: right now throws away rays that do not hit any domain owned by the current
  // rank
  t_filter.start();
  filterRaysLocally(rays);
  t_filter.stop();
  profiler.update(Profiler::Filter, t_filter.getElapsed());

  GVT_DEBUG(DBG_LOW, "tracing rays");

  // process domains until all rays are terminated
  bool all_done = false;
  int nqueue = 0;
  std::set<int> doms_to_send;
  int lastInstance = -1;
  // gvt::render::data::domain::AbstractDomain* dom = NULL;

  gvt::render::actor::RayVector moved_rays;
  moved_rays.reserve(1000);

  int instTarget = -1;
  size_t instTargetCount = 0;

  gvt::render::Adapter *adapter = 0;
  do {

    do {
      // process domain with most rays queued
      instTarget = -1;
      instTargetCount = 0;

      t_schedule.start();
      GVT_DEBUG(DBG_ALWAYS, "domain scheduler: selecting next instance, num queues: " << rayQ.size());
      // for (std::map<int, gvt::render::actor::RayVector>::iterator q = this->queue.begin(); q != this->queue.end();
      //      ++q) {
      for (auto &q : rayQ) {

        const bool inRank = mpiInstanceMap[q.first] == myRank;
        if (inRank && q.second.size() > instTargetCount) {
          instTargetCount = q.second.size();
          instTarget = q.first;
        }
      }
      t_schedule.stop();
      profiler.update(Profiler::Schedule, t_schedule.getElapsed());

#ifdef PROFILE_RAY_COUNTS
      profiler.addRayCountProc(instTargetCount);
#endif

      GVT_DEBUG(DBG_ALWAYS, "image scheduler: next instance: " << instTarget << ", rays: " << instTargetCount);

      if (instTarget >= 0) {
        t_adapter.start();
        gvt::render::Adapter *adapter = 0;
        // gvt::core::DBNodeH meshNode = instancenodes[instTarget]["meshRef"].deRef();

        gvt::render::data::primitives::Mesh *mesh = meshRef[instTarget];

        // TODO: Make cache generic needs to accept any kind of adpater

        // 'getAdapterFromCache' functionality
        auto it = adapterCache.find(mesh);
        if (it != adapterCache.end()) {
          adapter = it->second;
        } else {
          adapter = 0;
        }
        if (!adapter) {
          GVT_DEBUG(DBG_ALWAYS, "image scheduler: creating new adapter");
          switch (adapterType) {
#ifdef GVT_RENDER_ADAPTER_EMBREE
          case gvt::render::adapter::Embree:
            adapter = new gvt::render::adapter::embree::data::EmbreeMeshAdapter(mesh);
            break;
#endif
#ifdef GVT_RENDER_ADAPTER_MANTA
          case gvt::render::adapter::Manta:
            adapter = new gvt::render::adapter::manta::data::MantaMeshAdapter(mesh);
            break;
#endif
#ifdef GVT_RENDER_ADAPTER_OPTIX
          case gvt::render::adapter::Optix:
            adapter = new gvt::render::adapter::optix::data::OptixMeshAdapter(mesh);
            break;
#endif

#if defined(GVT_RENDER_ADAPTER_OPTIX) && defined(GVT_RENDER_ADAPTER_EMBREE)
          case gvt::render::adapter::Heterogeneous:
            adapter = new gvt::render::adapter::heterogeneous::data::HeterogeneousMeshAdapter(mesh);
            break;
#endif
          default:
            GVT_DEBUG(DBG_SEVERE, "image scheduler: unknown adapter type: " << adapterType);
          }

          adapterCache[mesh] = adapter;
        }
        t_adapter.stop();
        profiler.update(Profiler::Adapter, t_adapter.getElapsed());
        GVT_ASSERT(adapter != nullptr, "image scheduler: adapter not set");
        // end getAdapterFromCache concept

        GVT_DEBUG(DBG_ALWAYS, "image scheduler: calling process queue");
        {
          t_trace.start();
          moved_rays.reserve(rayQ[instTarget].size() * 10);
#ifdef GVT_USE_DEBUG
          boost::timer::auto_cpu_timer t("Tracing rays in adapter: %w\n");
#endif
          adapter->trace(rayQ[instTarget], moved_rays, instM[instTarget], instMinv[instTarget], instMinvN[instTarget],
                         lights);

          rayQ[instTarget].clear();

          t_trace.stop();
          profiler.update(Profiler::Trace, t_trace.getElapsed());
        }

        GVT_DEBUG(DBG_ALWAYS, "image scheduler: marching rays");

        t_shuffle.start();
        shuffleRays(moved_rays, instTarget);
        moved_rays.clear();
        t_shuffle.stop();
        profiler.update(Profiler::Shuffle, t_shuffle.getElapsed());
      }
    } while (instTarget != -1);

    all_done = transferRays();
  } while (!all_done);

  // add colors to the framebuffer
  t_composite.start();
  if (myRank == 0) {
    PixelGatherWork compositePixels;
    compositePixels.Broadcast(true, true);
  }
  t_composite.stop();
  profiler.update(Profiler::Composite, t_composite.getElapsed());
}

void MpiRenderer::shuffleRays(gvt::render::actor::RayVector &rays, int domID) {
  GVT_DEBUG(DBG_ALWAYS, "[" << myRank << "] Shuffle: start");
  GVT_DEBUG(DBG_ALWAYS, "[" << myRank << "] Shuffle: rays: " << rays.size());

  const size_t chunksize = MAX(2, rays.size() / (std::thread::hardware_concurrency() * 4));
  static gvt::render::data::accel::BVH &acc = *dynamic_cast<gvt::render::data::accel::BVH *>(acceleration);
  static tbb::simple_partitioner ap;
  tbb::parallel_for(tbb::blocked_range<gvt::render::actor::RayVector::iterator>(rays.begin(), rays.end(), chunksize),
                    [&](tbb::blocked_range<gvt::render::actor::RayVector::iterator> raysit) {
                      std::vector<gvt::render::data::accel::BVH::hit> hits =
                          acc.intersect<GVT_SIMD_WIDTH>(raysit.begin(), raysit.end(), domID);
                      std::map<int, gvt::render::actor::RayVector> local_queue;
                      for (size_t i = 0; i < hits.size(); i++) {
                        gvt::render::actor::Ray &r = *(raysit.begin() + i);
                        if (hits[i].next != -1) {
                          r.origin = r.origin + r.direction * (hits[i].t * 0.95f);
                          local_queue[hits[i].next].push_back(r);
                        } else if (r.type == gvt::render::actor::Ray::SHADOW && glm::length(r.color) > 0) {
                          tbb::mutex::scoped_lock fbloc(colorBufMutex[r.id % options.width]);
                          colorBuf[r.id] += r.color;
                        }
                      }
                      for (auto &q : local_queue) {

                        rayQMutex[q.first].lock();
                        rayQ[q.first].insert(rayQ[q.first].end(), std::make_move_iterator(local_queue[q.first].begin()),
                                             std::make_move_iterator(local_queue[q.first].end()));
                        rayQMutex[q.first].unlock();
                      }
                    },
                    ap);
  rays.clear();
}

// /**
//  * Given a queue of rays, intersects them against the accel structure
//  * to find out what instance they will hit next
//  */
// void MpiRenderer::shuffleRays(gvt::render::actor::RayVector &rays, gvt::core::DBNodeH instNode) {
//
//   GVT_DEBUG(DBG_ALWAYS, "[" << mpi.rank << "] Shuffle: start");
//   GVT_DEBUG(DBG_ALWAYS, "[" << mpi.rank << "] Shuffle: rays: " << rays.size());
//
//   const size_t raycount = rays.size();
//   const int domID = (instNode) ? instNode["id"].value().toInteger() : -1;
//   const gvt::render::data::primitives::Box3D domBB =
//       (instNode) ? *((Box3D *)instNode["bbox"].value().toULongLong()) : gvt::render::data::primitives::Box3D();
//
//   // tbb::parallel_for(tbb::blocked_range<gvt::render::actor::RayVector::iterator>(rays.begin(), rays.end()),
//   //                   [&](tbb::blocked_range<gvt::render::actor::RayVector::iterator> raysit) {
//     std::map<int, gvt::render::actor::RayVector> local_queue;
//     // for (gvt::render::actor::Ray &r : raysit) {
//     for (gvt::render::actor::RayVector::iterator it = rays.begin(); it != rays.end(); ++it) {
//       gvt::render::actor::Ray &r = *it;
//       if (domID != -1) {
//         float t = FLT_MAX;
//         if (r.domains.empty() && domBB.intersectDistance(r, t)) {
//           r.origin += r.direction * t;
//         }
//       }
//       if (r.domains.empty()) {
//         acceleration->intersect(r, r.domains);
//         boost::sort(r.domains);
//       }
//       if (!r.domains.empty() && (int)(*r.domains.begin()) == domID) {
//         r.domains.erase(r.domains.begin());
//       }
//       if (!r.domains.empty()) {
//         int firstDomainOnList = (*r.domains.begin());
//         r.domains.erase(r.domains.begin());
//         // tbb::mutex::scoped_lock sl(queue_mutex[firstDomainOnList]);
//         local_queue[firstDomainOnList].push_back(r);
//       } else if (instNode) {
//         tbb::mutex::scoped_lock fbloc(colorBufMutex[r.id % imageWidth]);
//         aggregatePixel(r.id, r.color);
//       }
//     }
//
//     std::vector<int> _doms;
//     std::transform(local_queue.begin(), local_queue.end(), std::back_inserter(_doms),
//                    [](const std::map<int, gvt::render::actor::RayVector>::value_type &pair) { return pair.first; });
//     while (!_doms.empty()) {
//       int dom = _doms.front();
//       _doms.erase(_doms.begin());
//       if (rayQMutex[dom].try_lock()) {
//         rayQ[dom].insert(rayQ[dom].end(), std::make_move_iterator(local_queue[dom].begin()),
//                              std::make_move_iterator(local_queue[dom].end()));
//         rayQMutex[dom].unlock();
//       } else {
//         _doms.push_back(dom);
//       }
//     }
//   // });
//   rays.clear();
// }

void MpiRenderer::localComposite() {
  const size_t size = options.width * options.height;
  const size_t chunksize = MAX(2, size / (std::thread::hardware_concurrency() * 4));
  static tbb::simple_partitioner ap;
  tbb::parallel_for(tbb::blocked_range<size_t>(0, size, chunksize),
                    [&](tbb::blocked_range<size_t> chunk) {
                      for (size_t i = chunk.begin(); i < chunk.end(); i++) image->Add(i, colorBuf[i]);
                    },
                    ap);
}

void MpiRenderer::gatherFramebuffers() {

  localComposite();
  // for (size_t i = 0; i < size; i++) image.Add(i, colorBuf[i]);

  // if (!mpi) return;

  size_t size = options.width * options.height;
  unsigned char *rgb = image->GetBuffer();

  int rgb_buf_size = 3 * size;

  unsigned char *bufs = (myRank == 0) ? new unsigned char[numRanks * rgb_buf_size] : NULL;

  // MPI_Barrier(MPI_COMM_WORLD);
  MPI_Gather(rgb, rgb_buf_size, MPI_UNSIGNED_CHAR, bufs, rgb_buf_size, MPI_UNSIGNED_CHAR, 0, MPI_COMM_WORLD);
  if (myRank == 0) {
    const size_t chunksize = MAX(2, size / (std::thread::hardware_concurrency() * 4));
    static tbb::simple_partitioner ap;
    tbb::parallel_for(tbb::blocked_range<size_t>(0, size, chunksize), [&](tbb::blocked_range<size_t> chunk) {

      for (int j = chunk.begin() * 3; j < chunk.end() * 3; j += 3) {
        for (size_t i = 1; i < numRanks; ++i) {
          int p = i * rgb_buf_size + j;
          // assumes black background, so adding is fine (r==g==b== 0)
          rgb[j + 0] += bufs[p + 0];
          rgb[j + 1] += bufs[p + 1];
          rgb[j + 2] += bufs[p + 2];
        }
      }
    });
#ifdef ENABLE_MESSAGE
    printf("[async mpi] Rank %d: writing result to file\n", myRank);
#endif
    image->Write();
  }
  delete[] bufs;
  pthread_mutex_lock(&imageReadyLock);
  imageReady = true;
  pthread_cond_signal(&imageReadyCond);
  pthread_mutex_unlock(&imageReadyLock);
}

// void MpiRenderer::compositePixels() {
// #ifdef ENABLE_TIMERS
//   Timer timer_composite;
// #endif
//   size_t size = options.width * options.height;
//   for (size_t i = 0; i < size; i++) image->Add(i, colorBuf[i]);
//
//   unsigned char *rgb = image->GetBuffer();
//   int rgb_buf_size = 3 * size;
//   unsigned char *bufs = (myRank == 0) ? new unsigned char[numRanks * rgb_buf_size] : NULL;
//
//   MPI_Gather(rgb, rgb_buf_size, MPI_UNSIGNED_CHAR, bufs, rgb_buf_size, MPI_UNSIGNED_CHAR, 0, MPI_COMM_WORLD);
//
//   if (myRank == 0) {
//     int nchunks = std::thread::hardware_concurrency() * 2;
//     int chunk_size = size / nchunks;
//     std::vector<std::pair<int, int> > chunks(nchunks);
//     std::vector<std::future<void> > futures;
//     for (int ii = 0; ii < nchunks - 1; ii++) {
//       chunks.push_back(std::make_pair(ii * chunk_size, ii * chunk_size + chunk_size));
//     }
//     int ii = nchunks - 1;
//     chunks.push_back(std::make_pair(ii * chunk_size, size));
//     for (auto &limit : chunks) {
//       futures.push_back(std::async(std::launch::async, [&]() {
//         // // std::pair<int,int> limit = std::make_pair(0,size);
//         // for (size_t i = 1; i < mpi.world_size; ++i) {
//         for (size_t i = 1; i < numRanks; ++i) {
//           for (int j = limit.first * 3; j < limit.second * 3; j += 3) {
//             int p = i * rgb_buf_size + j; // assumes black background, so adding is fine (r==g==b== 0)
//             rgb[j + 0] += bufs[p + 0];
//             rgb[j + 1] += bufs[p + 1];
//             rgb[j + 2] += bufs[p + 2];
//             // printf("%d (%f %f %f)\n", p, rgb[j], rgb[j+1], rgb[j+2]);
//           }
//         }
//       }));
//     }
//     for (std::future<void> &f : futures) {
//       f.wait();
//     }
//     printf("[async mpi] Rank %d: writing result to file\n", myRank);
//     image->Write(false);
//   }
//   delete[] bufs;
//
//   pthread_mutex_lock(&imageReadyLock);
//   imageReady = true;
//   pthread_cond_signal(&imageReadyCond);
//   pthread_mutex_unlock(&imageReadyLock);
// #ifdef ENABLE_TIMERS
//   timer_composite.stop();
//   profiler.update(Profiler::Composite, timer_composite.getElapsed());
// #endif
// }

void MpiRenderer::gatherTimes() {
  pthread_mutex_lock(&gatherTimesStartMutex);
  while (!gatherTimesStart) {
    pthread_cond_wait(&gatherTimesStartCond, &gatherTimesStartMutex);
  }
  gatherTimesStart = false;
  pthread_mutex_unlock(&gatherTimesStartMutex);

  if (myRank == 0) {
    profiler.gtimes.resize(numRanks * Profiler::NumTimers);
#ifdef PROFILE_RAY_COUNTS
    profiler.grays.resize(numRanks);
#endif
  }

  MPI_Gather(static_cast<const void *>(&profiler.times[0]), Profiler::NumTimers, MPI_DOUBLE,
             static_cast<void *>(&profiler.gtimes[0]), Profiler::NumTimers, MPI_DOUBLE, 0, MPI_COMM_WORLD);

#ifdef PROFILE_RAY_COUNTS
  MPI_Gather(static_cast<const void *>(&profiler.rays), sizeof(Profiler::RayCounts), MPI_BYTE,
             static_cast<void *>(&profiler.grays[0]), sizeof(Profiler::RayCounts), MPI_BYTE, 0, MPI_COMM_WORLD);
#endif

  if (myRank == 0) {
    profiler.print(options.numFrames, numRanks);
    // for (int i = 0; i < profiler.gtimes.size(); ++i) printf("[%d]=%f\n", i, profiler.gtimes[i]);
  }

  pthread_mutex_lock(&gatherTimesDoneMutex);
  gatherTimesDone = true;
  pthread_cond_signal(&gatherTimesDoneCond);
  pthread_mutex_unlock(&gatherTimesDoneMutex);
}
