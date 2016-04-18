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
#include "gvt/render/data/Dataset.h"
#include "gvt/render/data/Domains.h"
#include <algorithm>
#include <set>
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
#define ENABLE_TIMERS

using namespace std;
using namespace gvt::render;
using namespace gvt::core;
using namespace gvt::core::math;
using namespace gvt::core::mpi;
using namespace gvt::render::data::scene;
using namespace gvt::render::schedule;
using namespace gvt::render::data::primitives;
using namespace gvt::render::unit;
using namespace gvt::render::actor;

MpiRenderer::MpiRenderer(int *argc, char ***argv)
    : Application(argc, argv), camera(NULL), image(NULL), tileLoadBalancer(NULL), voter(NULL), adapter(NULL),
      acceleration(NULL), rayQueueMutex(NULL), colorBufMutex(NULL) {}

MpiRenderer::~MpiRenderer() {
  if (camera != NULL) delete camera;
  if (image != NULL) delete image;
  if (tileLoadBalancer != NULL) delete tileLoadBalancer;
  if (adapter != NULL) delete adapter;
  if (acceleration) delete acceleration;
  if (rayQueueMutex) delete[] rayQueueMutex;
  if (colorBufMutex) delete[] colorBufMutex;
  if (voter) delete voter;
}

void MpiRenderer::printUsage(const char *argv) {
  printf("Usage : %s [-h] [-a <adapter>] [-n <x y z>] [-p] [-s <scheduler>] [-W <image_width>] [-H "
         "<image_height>] [-N <num_frames>]\n",
         argv);
  printf("  -h, --help\n");
  printf("  -a, --adapter <embree | manta | optix> (default: embree)\n");
  printf("  -s, --scheduler <0-3> (default: 1)\n");
  printf("      0: AsyncImage, 1: AysncDomain, 2: SyncImage, 3:SyncDomain\n");
  printf("  -n, --num-instances <x, y, z> specify the number of instances in each direction (default: 1 1 1). "
         "effective only with obj.\n");
  printf("  -p, --ply use ply models\n");
  printf("  -W, --width <image_width> (default: 1280)\n");
  printf("  -H, --height <image_height> (default: 720)\n");
  printf("  -N, --num-frames <num_frames> (default: 1)\n");
}

void MpiRenderer::parseCommandLine(int argc, char **argv) {
  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
      printUsage(argv[0]);
      exit(1);
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
    } else {
      printf("error: %s not defined\n", argv[i]);
      exit(1);
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
  Point4f eye(0.0, 0.5, 1.2, 1.0);
  Point4f focus(0.0, 0.0, 0.0, 1.0);
  Vector4f upVector(0.0, 1.0, 0.0, 0.0);
  float fov = (45.0 * M_PI / 180.0);
  if (options.ply) {
    eye = Point4f(512.0, 512.0, 4096.0, 1.0);
    focus = Point4f(512.0, 512.0, 0.0, 1.0);
    upVector = Vector4f(0.0, 1.0, 0.0, 0.0);
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
  std::cout << "instance node size: " << instanceNodes.size() << "\ndata node size: " << dataNodes.size() << "\n";
#endif

  // create a map of instances to mpi rank
  for (size_t i = 0; i < instanceNodes.size(); ++i) {
    gvt::core::DBNodeH meshNode = instanceNodes[i]["meshRef"].deRef();
    size_t dataIdx = -1;
    for (size_t d = 0; d < dataNodes.size(); ++d) {
      if (dataNodes[d].UUID() == meshNode.UUID()) {
        dataIdx = d;
        break;
      }
    }
    // NOTE: mpi-data(domain) assignment strategy
    int ownerRank = static_cast<int>(dataIdx) % GetSize();
    GVT_DEBUG(DBG_ALWAYS, "[" << GetRank() << "] domain scheduler: instId: " << i << ", dataIdx: " << dataIdx
                              << ", target mpi node: " << ownerRank << ", world size: " << GetSize());
#ifdef DEBUG_MPI_RENDERER
    std::cout << "[" << GetRank() << "] domain scheduler: instId: " << i << ", dataIdx: " << dataIdx
              << ", target mpi node: " << ownerRank << ", world size: " << GetSize() << "\n";
#endif
    GVT_ASSERT(dataIdx != (size_t) - 1, "domain scheduler: could not find data node");
    instanceRankMap[instanceNodes[i].UUID()] = ownerRank;
  }
}

void MpiRenderer::setupCommon() {
  tbb::task_scheduler_init init(std::thread::hardware_concurrency());
  auto renderContext = gvt::render::RenderContext::instance();
  if (renderContext == NULL) {
    std::cout << "Something went wrong initializing the context" << std::endl;
    exit(1);
  }
  root = renderContext->getRootNode();
  instanceNodes = root["Instances"].getChildren();
  GVT_DEBUG(DBG_ALWAYS, "num instances: " << instanceNodes.size());
  imageWidth = root["Film"]["width"].value().toInteger();
  imageHeight = root["Film"]["height"].value().toInteger();
  framebuffer.resize(imageWidth * imageHeight);
  acceleration = new gvt::render::data::accel::BVH(instanceNodes);
  rayQueueMutex = new tbb::mutex[instanceNodes.size()];
  colorBufMutex = new tbb::mutex[imageWidth];
  numRanks = GetSize();
  myRank = GetRank();

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
    voter = new Voter(numRanks, myRank, &rayQueue);
  }
}

void MpiRenderer::setupSyncImage() {
  setupCommon();
}

void MpiRenderer::setupSyncDomain() {
  setupCommon();
}

void MpiRenderer::freeRender() {
#ifdef DEBUG_MPI_RENDERER
  printf("Rank %d: MpiRenderer::freeRender. cleaning up.\n", GetRank());
#endif
  delete acceleration;
  delete[] rayQueueMutex;
  delete[] colorBufMutex;
  if (voter) delete voter;
}

void MpiRenderer::render() {
  switch (options.schedulerType) {
  case MpiRendererOptions::AsyncImage: {
    renderAsyncImage();
  } break;
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

void MpiRenderer::renderAsyncImage() {
  setupAsyncImage();
  GVT_ASSERT(options.numFrames == 1, "multiple frames not supported yet for image scheduler");
  if (myRank == 0) printf("[async mpi] starting image scheduler using %d processes\n", GetSize());

  RequestWork::Register();
  TileWork::Register();
  ImageTileWork::Register();
  PixelWork::Register();

  Application::Start();
  if (myRank == rank::Server) {
    initServer();
  }
  RequestWork request;
  request.setSourceRank(GetRank());
  request.Send(rank::Server);
  Application::Wait();
}

void MpiRenderer::renderAsyncDomain() {
  setupAsyncDomain();
  if (myRank == 0) printf("[async mpi] starting domain scheduler using %d processes\n", GetSize());

  RayTransferWork::Register();
  VoteWork::Register();
  PixelGatherWork::Register();
#ifdef ENABLE_TIMERS
  TimeGatherWork::Register();
#endif
  Application::Start();

  image = new Image(imageWidth, imageHeight, "image");

#ifdef ENABLE_TIMERS
  Timer timer_total;
  Timer timer_wait_image;
#endif
  for (int i = 0; i < options.numFrames; ++i) {
    printf("[async mpi] Rank %d: frame %d start\n", myRank, i);
    runDomainTracer();

#ifdef ENABLE_TIMERS
    timer_wait_image.start();
#endif
    pthread_mutex_lock(&imageReadyLock);
    while (!imageReady) pthread_cond_wait(&imageReadyCond, &imageReadyLock);
    imageReady = false;
    pthread_mutex_unlock(&imageReadyLock);
#ifdef ENABLE_TIMERS
    timer_wait_image.stop();
    profiler.update(Profiler::WaitImage, timer_wait_image.getElapsed());
#endif
    if (numRanks > 1) voter->reset();
    printf("[async mpi] Rank %d: frame %d done\n", myRank, i);
  }
#ifdef ENABLE_TIMERS
  timer_total.stop();
  profiler.update(Profiler::Total, timer_total.getElapsed());
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
#endif
  Application::Kill();
}

void MpiRenderer::renderSyncImage() {
  setupSyncImage();
  GVT_ASSERT(options.numFrames == 1, "multiple frames not supported yet for schedulers with synchronous MPI");
  camera->AllocateCameraRays();
  camera->generateRays();
  image = new Image(imageWidth, imageHeight, "image");
  if (myRank == 0) printf("[sync mpi] starting image scheduler without the mpi layer using %d processes\n", GetSize());
  gvt::render::algorithm::Tracer<ImageScheduler>(camera->rays, *image)();
  image->Write();
  Quit::Register();
  Start();
  if (myRank == 0) {
    Quit quit;
    quit.Broadcast(true, true);
  }
}

void MpiRenderer::renderSyncDomain() {
  setupSyncDomain();
  GVT_ASSERT(options.numFrames == 1, "multiple frames not supported yet for schedulers with synchronous MPI");
  camera->AllocateCameraRays();
  camera->generateRays();
  image = new Image(imageWidth, imageHeight, "image");
  if (myRank == 0) printf("[sync mpi] starting domain scheduler without the mpi layer using %d processes\n", GetSize());
  gvt::render::algorithm::Tracer<DomainScheduler>(camera->rays, *image)();
  image->Write();
  Quit::Register();
  Start();
  if (myRank == 0) {
    Quit quit;
    quit.Broadcast(true, true);
  }
}

void MpiRenderer::initServer() {
  // TODO (hpark): For now, equally divide the image
  // try coarser/finer granularity later
  int schedType = root["Schedule"]["type"].value().toInteger();
  // int numRanks = GetSize();
  int numWorkers = numRanks;
  int granularity = numRanks;

  tileLoadBalancer = new TileLoadBalancer(schedType, imageWidth, imageHeight, granularity, numWorkers);
  image = new Image(imageWidth, imageHeight, "image");
  pendingPixelCount = imageWidth * imageHeight;

  pthread_mutex_lock(&serverReadyLock);
  serverReady = true;
  pthread_cond_signal(&serverReadyCond);
  pthread_mutex_unlock(&serverReadyLock);
}

void MpiRenderer::aggregatePixel(int pixelId, const GVT_COLOR_ACCUM &addend) {
  GVT_COLOR_ACCUM &color = framebuffer[pixelId];
  color.add(addend);
  color.rgba[3] = 1.f;
  color.clamp();
}

bool MpiRenderer::transferRays() {
#ifdef ENABLE_TIMERS
  Timer timer_vote;
  Timer timer_transfer;
#endif
  bool done;
  if (numRanks > 1) {
    sendRays();
    receiveRays();
#ifdef ENABLE_TIMERS
    timer_transfer.stop();
    profiler.update(Profiler::Transfer, timer_transfer.getElapsed());
    timer_vote.start();
#endif
    done = voter->updateState();
#ifdef ENABLE_TIMERS
    timer_vote.stop();
    profiler.update(Profiler::Vote, timer_vote.getElapsed());
#endif
  } else {
    done = rayQueue.empty();
#ifdef ENABLE_TIMERS
    timer_transfer.stop();
    profiler.update(Profiler::Transfer, timer_transfer.getElapsed());
#endif
  }
  return done;
}

void MpiRenderer::sendRays() {
  for (auto &q : rayQueue) {
    int instance = q.first;
    RayVector &rays = q.second;
    int ownerRank = getInstanceOwner(instance);
    size_t numRaysToSend = rays.size();
    if (ownerRank != myRank && numRaysToSend > 0) {
      voter->addNumPendingRays(numRaysToSend);
      RayTransferWork work;
      work.setup(RayTransferWork::Request, myRank, instance, &rays);
      work.Send(ownerRank);
#ifdef DEBUG_RAYTX
      printf("rank %d: sent %lu rays instance %d\n", myRank, numRaysToSend, instance);
#endif
    }
  }
}

void MpiRenderer::receiveRays() {
  pthread_mutex_lock(&rayTransferBufferLock);
  for (size_t i = 0; i < rayTransferBuffer.size(); ++i) {
    RayTransferWork *raytx = rayTransferBuffer[i];
    raytx->copyIncomingRays(&rayQueue);

    RayTransferWork grant;
    grant.setup(RayTransferWork::Grant, myRank, raytx->getNumRays());
    grant.Send(raytx->getSenderRank());
#ifdef DEBUG_RAYTX
    printf("rank %d: recved %d rays instance %d \n", myRank, raytx->getNumRays(), raytx->getInstanceId());
#endif
    delete raytx;
  }
  rayTransferBuffer.clear(); // TODO: avoid this

  pthread_mutex_unlock(&rayTransferBufferLock);
}

void MpiRenderer::bufferRayTransferWork(RayTransferWork *work) {
  pthread_mutex_lock(&rayTransferBufferLock);
  rayTransferBuffer.push_back(work); // TODO: avoid resizing
  pthread_mutex_unlock(&rayTransferBufferLock);
}

void MpiRenderer::bufferVoteWork(VoteWork *work) { voter->bufferVoteWork(work); }
void MpiRenderer::voteForResign(int senderRank, unsigned int timeStamp) { voter->voteForResign(senderRank, timeStamp); }
void MpiRenderer::voteForNoWork(int senderRank, unsigned int timeStamp) { voter->voteForNoWork(senderRank, timeStamp); }
void MpiRenderer::applyRayTransferResult(int numRays) { voter->subtractNumPendingRays(numRays); }
void MpiRenderer::applyVoteResult(int voteType, unsigned int timeStamp) { voter->applyVoteResult(voteType, timeStamp); }

void MpiRenderer::copyIncomingRays(int instanceId, const gvt::render::actor::RayVector *incomingRays) {
  pthread_mutex_lock(&rayTransferMutex);
  if (rayQueue.find(instanceId) != rayQueue.end()) {
    rayQueue[instanceId].insert(rayQueue[instanceId].end(), incomingRays->begin(), incomingRays->end());
  } else {
    rayQueue[instanceId] = *incomingRays;
  }
  pthread_mutex_unlock(&rayTransferMutex);
}

void MpiRenderer::runDomainTracer() {
#ifdef ENABLE_TIMERS
  Timer timer_primary;
#endif
  RayVector rays;
  generatePrimaryRays(rays);
#ifdef ENABLE_TIMERS
  timer_primary.stop();
  profiler.update(Profiler::Primary, timer_primary.getElapsed());
#endif
  domainTracer(rays);
}

void MpiRenderer::generatePrimaryRays(RayVector &rays) {
  const int imageWidth = options.width;
  const int imageHeight = options.height;
  const int tileW = options.width;
  const int tileH = options.height;
  const int startX = 0;
  const int startY = 0;

  gvt::core::DBNodeH root = gvt::render::RenderContext::instance()->getRootNode();
  Point4f eye = root["Camera"]["eyePoint"].value().toPoint4f();
  float fov = root["Camera"]["fov"].value().toFloat();
  const AffineTransformMatrix<float> &cameraToWorld = camera->getCameraToWorld();

  rays.resize(tileW * tileH);

  // Generate rays direction in camera space and transform to world space.
  int i, j;
  int tilePixelId, imagePixelId;
  float aspectRatio = float(imageWidth) / float(imageHeight);
  float x, y;
  // these basis directions are scaled by the aspect ratio and
  // the field of view.
  Vector4f camera_vert_basis_vector = Vector4f(0, 1, 0, 0) * tan(fov * 0.5);
  Vector4f camera_horiz_basis_vector = Vector4f(1, 0, 0, 0) * tan(fov * 0.5) * aspectRatio;
  Vector4f camera_normal_basis_vector = Vector4f(0, 0, 1, 0);
  Vector4f camera_space_ray_direction;

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
      camera_space_ray_direction =
          camera_normal_basis_vector + x * camera_horiz_basis_vector + y * camera_vert_basis_vector;
      // transform ray to world coordinate space;
      ray.setDirection(cameraToWorld * camera_space_ray_direction.normalize());
      ray.depth = 0;
    }
  }
}

void MpiRenderer::filterRaysLocally(RayVector &rays) {
  auto nullNode = gvt::core::DBNodeH(); // temporary workaround until shuffleRays is fully replaced
  GVT_DEBUG(DBG_ALWAYS, "image scheduler: filter locally non mpi: " << rays.size());
  shuffleRays(rays, nullNode);
}

void MpiRenderer::domainTracer(RayVector &rays) {
#ifdef ENABLE_TIMERS
  Timer timer_filter;
  Timer timer_schedule;
  Timer timer_trace;
  Timer timer_shuffle;
#endif

  GVT_DEBUG(DBG_ALWAYS, "domain scheduler: starting, num rays: " << rays.size());
  int adapterType = root["Schedule"]["adapter"].value().toInteger();
  long domain_counter = 0;

#ifdef ENABLE_TIMERS
  timer_filter.start();
#endif
  filterRaysLocally(rays);
#ifdef ENABLE_TIMERS
  timer_filter.stop();
  profiler.update(Profiler::Filter, timer_filter.getElapsed())
#endif
      GVT_DEBUG(DBG_LOW, "tracing rays");
  // process domains until all rays are terminated
  bool all_done = false;
  // std::set<int> doms_to_send;
  int lastInstance = -1;
  // gvt::render::data::domain::AbstractDomain* dom = NULL;
  gvt::render::actor::RayVector moved_rays;
  moved_rays.reserve(1000);
  int instTarget = -1;
  size_t instTargetCount = 0;
  // gvt::render::Adapter *adapter = 0;

  while (!all_done) {
    // pthread_mutex_lock(rayTransferMutex);
    if (!rayQueue.empty()) {
#ifdef ENABLE_TIMERS
      timer_schedule.start();
#endif
      // process domain assigned to this proc with most rays queued
      // if there are queues for instances that are not assigned
      // to the current rank, erase those entries
      instTarget = -1;
      instTargetCount = 0;
      std::vector<int> to_del;
      GVT_DEBUG(DBG_ALWAYS, "domain scheduler: selecting next instance, num queues: " << rayQueue.size());
      for (auto &q : rayQueue) {
        const bool inRank = (getInstanceOwner(q.first) == myRank);
        if (q.second.empty() || !inRank) {
          to_del.push_back(q.first);
          continue;
        }
        if (inRank && q.second.size() > instTargetCount) {
          instTargetCount = q.second.size();
          instTarget = q.first;
        }
      }
      // erase empty queues
      for (int instId : to_del) {
        GVT_DEBUG(DBG_ALWAYS, "rank[" << myRank << "] DOMAINTRACER: deleting rayQueue for instance " << instId);
        rayQueue.erase(instId);
      }
      if (instTarget == -1) {
#ifdef ENABLE_TIMERS
        timer_schedule.stop();
        profiler.update(Profiler::Schedule, timer_schedule.getElapsed());
#endif
        continue;
      }
#ifdef ENABLE_TIMERS
      else {
        timer_schedule.stop();
        profiler.update(Profiler::Schedule, timer_schedule.getElapsed());
      }
#endif

      GVT_DEBUG(DBG_ALWAYS, "domain scheduler: next instance: " << instTarget << ", rays: " << instTargetCount << " ["
                                                                << myRank << "]");
      // doms_to_send.clear();
      // pnav: use this to ignore domain x:        int domi=0;if (0)
      if (instTarget >= 0) {
        GVT_DEBUG(DBG_LOW, "Getting instance " << instTarget);
        // gvt::render::Adapter *adapter = 0;
        gvt::core::DBNodeH meshNode = getMeshNode(instTarget);
        if (instTarget != lastInstance) {
          // TODO: before we would free the previous domain before loading the next
          // this can be replicated by deleting the adapter
          // if (!adapter) {
          //   printf("trying to delete null adapter instTarget %d lastInstance %d\n", instTarget, lastInstance);
          //   exit(1);
          // }
          if (!adapter) {
            delete adapter;
            adapter = 0;
          }
        }
        // track domains loaded
        if (instTarget != lastInstance) {
          ++domain_counter;
          lastInstance = instTarget;
          //
          // 'getAdapterFromCache' functionality
          if (!adapter) {
            GVT_DEBUG(DBG_ALWAYS, "domain scheduler: creating new adapter");
            switch (adapterType) {
#ifdef GVT_RENDER_ADAPTER_EMBREE
            case gvt::render::adapter::Embree:
              adapter = new gvt::render::adapter::embree::data::EmbreeMeshAdapter(meshNode);
              break;
#endif
#ifdef GVT_RENDER_ADAPTER_MANTA
            case gvt::render::adapter::Manta:
              adapter = new gvt::render::adapter::manta::data::MantaMeshAdapter(meshNode);
              break;
#endif
#ifdef GVT_RENDER_ADAPTER_OPTIX
            case gvt::render::adapter::Optix:
              adapter = new gvt::render::adapter::optix::data::OptixMeshAdapter(meshNode);
              break;
#endif
#if defined(GVT_RENDER_ADAPTER_OPTIX) && defined(GVT_RENDER_ADAPTER_EMBREE)
            case gvt::render::adapter::Heterogeneous:
              adapter = new gvt::render::adapter::heterogeneous::data::HeterogeneousMeshAdapter(meshNode);
              break;
#endif
            default:
              GVT_DEBUG(DBG_SEVERE, "domain scheduler: unknown adapter type: " << adapterType);
            }
            // adapterCache[meshNode.UUID()] = adapter; // note: cache logic
            // comes later when we implement hybrid
          }
          // end 'getAdapterFromCache' concept
          //
        }
        GVT_ASSERT(adapter != nullptr, "domain scheduler: adapter not set");
        GVT_DEBUG(DBG_ALWAYS, "[" << myRank << "] domain scheduler: calling process rayQueue");
        gvt::core::DBNodeH instNode = getInstanceNode(instTarget);
        {
#ifdef ENABLE_TIMERS
          timer_trace.start();
#endif
          moved_rays.reserve(rayQueue[instTarget].size() * 10);
          if (!adapter) {
            printf("nulll adapter detected\n");
            exit(1);
          }
          adapter->trace(rayQueue[instTarget], moved_rays, instNode);
          rayQueue[instTarget].clear();
#ifdef ENABLE_TIMERS
          timer_trace.stop();
          profiler.update(Profiler::Trace, timer_trace.getElapsed());
#endif
        }
#ifdef ENABLE_TIMERS
        timer_shuffle.start();
#endif
        shuffleRays(moved_rays, instNode);
        moved_rays.clear();
#ifdef ENABLE_TIMERS
        timer_shuffle.stop();
        profiler.update(Profiler::Shuffle, timer_shuffle.getElapsed());
#endif
      }
    } // if (!rayQueue.empty()) {
    // done with current domain, send off rays to their proper processors.
    GVT_DEBUG(DBG_ALWAYS, "Rank [ " << myRank << "]  calling SendRays");
    all_done = transferRays();
    // pthread_mutex_unlock(rayTransferMutex);
  } // while (!all_done) {

  if (myRank == 0) {
    PixelGatherWork compositePixels;
    compositePixels.Broadcast(true, true);
  }
}

/**
 * Given a queue of rays, intersects them against the accel structure
 * to find out what instance they will hit next
 */
void MpiRenderer::shuffleRays(gvt::render::actor::RayVector &rays, gvt::core::DBNodeH instNode) {
  GVT_DEBUG(DBG_ALWAYS, "[" << mpi.rank << "] Shuffle: start");
  GVT_DEBUG(DBG_ALWAYS, "[" << mpi.rank << "] Shuffle: rays: " << rays.size());

  const size_t raycount = rays.size();
  const int domID = (instNode) ? instNode["id"].value().toInteger() : -1;
  const gvt::render::data::primitives::Box3D domBB =
      (instNode) ? *((Box3D *)instNode["bbox"].value().toULongLong()) : gvt::render::data::primitives::Box3D();

  // tbb::parallel_for(tbb::blocked_range<gvt::render::actor::RayVector::iterator>(rays.begin(), rays.end()),
  //                   [&](tbb::blocked_range<gvt::render::actor::RayVector::iterator> raysit) {
    std::map<int, gvt::render::actor::RayVector> local_queue;
    // for (gvt::render::actor::Ray &r : raysit) {
    for (gvt::render::actor::RayVector::iterator it = rays.begin(); it != rays.end(); ++it) {
      gvt::render::actor::Ray &r = *it;
      if (domID != -1) {
        float t = FLT_MAX;
        if (r.domains.empty() && domBB.intersectDistance(r, t)) {
          r.origin += r.direction * t;
        }
      }
      if (r.domains.empty()) {
        acceleration->intersect(r, r.domains);
        boost::sort(r.domains);
      }
      if (!r.domains.empty() && (int)(*r.domains.begin()) == domID) {
        r.domains.erase(r.domains.begin());
      }
      if (!r.domains.empty()) {
        int firstDomainOnList = (*r.domains.begin());
        r.domains.erase(r.domains.begin());
        // tbb::mutex::scoped_lock sl(queue_mutex[firstDomainOnList]);
        local_queue[firstDomainOnList].push_back(r);
      } else if (instNode) {
        tbb::mutex::scoped_lock fbloc(colorBufMutex[r.id % imageWidth]);
        aggregatePixel(r.id, r.color);
      }
    }

    std::vector<int> _doms;
    std::transform(local_queue.begin(), local_queue.end(), std::back_inserter(_doms),
                   [](const std::map<int, gvt::render::actor::RayVector>::value_type &pair) { return pair.first; });
    while (!_doms.empty()) {
      int dom = _doms.front();
      _doms.erase(_doms.begin());
      if (rayQueueMutex[dom].try_lock()) {
        rayQueue[dom].insert(rayQueue[dom].end(), std::make_move_iterator(local_queue[dom].begin()),
                             std::make_move_iterator(local_queue[dom].end()));
        rayQueueMutex[dom].unlock();
      } else {
        _doms.push_back(dom);
      }
    }
  // });
  rays.clear();
}

void MpiRenderer::compositePixels() {
#ifdef ENABLE_TIMERS
  Timer timer_composite;
#endif
  size_t size = options.width * options.height;
  for (size_t i = 0; i < size; i++) image->Add(i, framebuffer[i]);

  unsigned char *rgb = image->GetBuffer();
  int rgb_buf_size = 3 * size;
  unsigned char *bufs = (myRank == 0) ? new unsigned char[numRanks * rgb_buf_size] : NULL;

  MPI_Gather(rgb, rgb_buf_size, MPI_UNSIGNED_CHAR, bufs, rgb_buf_size, MPI_UNSIGNED_CHAR, 0, MPI_COMM_WORLD);

  if (myRank == 0) {
    int nchunks = std::thread::hardware_concurrency() * 2;
    int chunk_size = size / nchunks;
    std::vector<std::pair<int, int> > chunks(nchunks);
    std::vector<std::future<void> > futures;
    for (int ii = 0; ii < nchunks - 1; ii++) {
      chunks.push_back(std::make_pair(ii * chunk_size, ii * chunk_size + chunk_size));
    }
    int ii = nchunks - 1;
    chunks.push_back(std::make_pair(ii * chunk_size, size));
    for (auto &limit : chunks) {
      futures.push_back(std::async(std::launch::async, [&]() {
        // // std::pair<int,int> limit = std::make_pair(0,size);
        // for (size_t i = 1; i < mpi.world_size; ++i) {
        for (size_t i = 1; i < numRanks; ++i) {
          for (int j = limit.first * 3; j < limit.second * 3; j += 3) {
            int p = i * rgb_buf_size + j; // assumes black background, so adding is fine (r==g==b== 0)
            rgb[j + 0] += bufs[p + 0];
            rgb[j + 1] += bufs[p + 1];
            rgb[j + 2] += bufs[p + 2];
            // printf("%d (%f %f %f)\n", p, rgb[j], rgb[j+1], rgb[j+2]);
          }
        }
      }));
    }
    for (std::future<void> &f : futures) {
      f.wait();
    }
    printf("[async mpi] Rank %d: writing result to file\n", myRank);
    image->Write(false);
  }
  delete[] bufs;

  pthread_mutex_lock(&imageReadyLock);
  imageReady = true;
  pthread_cond_signal(&imageReadyCond);
  pthread_mutex_unlock(&imageReadyLock);
#ifdef ENABLE_TIMERS
  timer_composite.stop();
  profiler.update(Profiler::Composite, timer_composite.getElapsed());
#endif
}

void MpiRenderer::gatherTimes() {
  pthread_mutex_lock(&gatherTimesStartMutex);
  while (!gatherTimesStart) {
    pthread_cond_wait(&gatherTimesStartCond, &gatherTimesStartMutex);
  }
  gatherTimesStart = false;
  pthread_mutex_unlock(&gatherTimesStartMutex);

  if (myRank == 0) {
    profiler.gtimes.resize(numRanks * Profiler::Size);
  }

  MPI_Gather(static_cast<const void *>(&profiler.times[0]), Profiler::Size, MPI_DOUBLE,
             static_cast<void *>(&profiler.gtimes[0]), Profiler::Size, MPI_DOUBLE, 0, MPI_COMM_WORLD);

  if (myRank == 0) {
    profiler.print(options.numFrames, numRanks);
    // for (int i = 0; i < profiler.gtimes.size(); ++i) printf("[%d]=%f\n", i, profiler.gtimes[i]);
  }

  pthread_mutex_lock(&gatherTimesDoneMutex);
  gatherTimesDone = true;
  pthread_cond_signal(&gatherTimesDoneCond);
  pthread_mutex_unlock(&gatherTimesDoneMutex);
}
