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

#include "gvt/render/unit/DomainTileWork.h"
#include "gvt/render/unit/ImageTileWork.h"
#include "gvt/render/unit/PixelGatherWork.h"
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

// #define DEBUG_MPI_RENDERER
// #define DEBUG_RAYTX

using namespace std;
using namespace gvt::render;
using namespace gvt::core;
using namespace gvt::core::math;
using namespace gvt::core::mpi;
using namespace gvt::render::data::scene;
using namespace gvt::render::schedule;
using namespace gvt::render::data::primitives;
using namespace gvt::render::unit;

MpiRenderer::MpiRenderer(int *argc, char ***argv)
    : Application(argc, argv), camera(NULL), image(NULL), tileLoadBalancer(NULL), voter(NULL) {

}

MpiRenderer::~MpiRenderer() {
  if (camera != NULL) delete camera;
  if (image != NULL) delete image;
  if (tileLoadBalancer != NULL) delete tileLoadBalancer;
}

void MpiRenderer::printUsage(const char *argv) {
  printf(
      "Usage : %s [-h] [-a <adapter>] [-d] [-n <x y z>] [-p] [-s <scheduler>] [-W <image_width>] [-H <image_height>]\n",
      argv);
  printf("  -h, --help\n");
  printf("  -a, --adapter <embree | manta | optix> (default: embree)\n");
  printf("  -s, --scheduler <domain | image> (default: domain)\n");
  printf("  -d, --disable-aync-mpi\n");
  printf("  -n, --num-instances <x, y, z> specify the number of instances in each direction (default: 1 1 1). "
         "effective only with obj.\n");
  printf("  -p, --ply use ply models\n");
  printf("  -W, --width <image_width> (default: 1280)\n");
  printf("  -H, --height <image_height> (default: 1280)\n");
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
      ++i;
      if (strcmp(argv[i], "domain") == 0) {
        options.schedulerType = gvt::render::scheduler::Domain;
      } else if (strcmp(argv[i], "image") == 0) {
        options.schedulerType = gvt::render::scheduler::Image;
      } else {
        printf("error: %s not defined\n", argv[i]);
        exit(1);
      }
    } else if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--disable-async-mpi") == 0) {
      options.asyncMpi = false;
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
    GVT_ASSERT(dataIdx != (size_t)-1, "domain scheduler: could not find data node");
    instanceRankMap[instanceNodes[i].UUID()] = ownerRank;
  }
}

void MpiRenderer::setupRender() {
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

  int schedType = root["Schedule"]["type"].value().toInteger();

  numRanks = GetSize();
  myRank = GetRank();

  if (schedType == scheduler::Image) {
    if (GetRank() == rank::Server) {
      serverReady = false;
      pthread_mutex_init(&serverReadyLock, NULL);
      pthread_cond_init(&serverReadyCond, NULL);
    }
  } else if (schedType == scheduler::Domain) {
    initInstanceRankMap();
    pthread_mutex_init(&rayTransferBufferLock, NULL);
    pthread_mutex_init(&rayTransferMutex, NULL);
    if (numRanks > 1) {
      voter = new Voter(numRanks, myRank, &rayQueue);
    }
  }
  // image write
  imageReady = false;
  pthread_mutex_init(&imageReadyLock, NULL);
  pthread_cond_init(&imageReadyCond, NULL);
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
  setupRender();
  int schedType = root["Schedule"]["type"].value().toInteger();
  int rank = GetRank();

  if (options.asyncMpi) {
    // TODO (hpark):
    // collapse the following two if-else blocks into a single block
    if (schedType == scheduler::Image) { // image scheduler with mpi layer

      if (rank == 0) printf("[async mpi] starting image scheduler using %d processes\n", GetSize());

      RequestWork::Register();
      TileWork::Register();
      ImageTileWork::Register();
      PixelWork::Register();

      Start();

      if (rank == rank::Server) {
        initServer();
      }

      RequestWork request;
      request.setSourceRank(GetRank());
      request.Send(rank::Server);

      Wait();

    } else { // domain scheduler with mpi layer
      if (rank == 0) printf("[async mpi] starting domain scheduler using %d processes\n", GetSize());

      DomainTileWork::Register();
      RayTransferWork::Register();
      VoteWork::Register();
      PixelGatherWork::Register();

      Start();

      image = new Image(imageWidth, imageHeight, "image");
      DomainTileWork work;
      work.setTileSize(0, 0, imageWidth, imageHeight);

      const int numFrames = 1;
      for (int i = 0; i < numFrames; ++i) {
        work.Action();
        pthread_mutex_lock(&imageReadyLock);
        while (!this->imageReady) {
          pthread_cond_wait(&imageReadyCond, &imageReadyLock);
        }
        imageReady = false;
        pthread_mutex_unlock(&imageReadyLock);
        printf("[async mpi] domain scheduler frame %d done\n", i);
      }
      Kill();
    }
    freeRender();
  } else { // without mpi layer
    camera->AllocateCameraRays();
    camera->generateRays();
    image = new Image(imageWidth, imageHeight, "image");

    if (schedType == scheduler::Image) {
      if (rank == 0)
        printf("[sync mpi] starting image scheduler without the mpi layer using %d processes\n", GetSize());
      gvt::render::algorithm::Tracer<ImageScheduler>(camera->rays, *image)();
    } else {
      if (rank == 0)
        printf("[sync mpi] starting domain scheduler without the mpi layer using %d processes\n", GetSize());
      gvt::render::algorithm::Tracer<DomainScheduler>(camera->rays, *image)();
    }

    image->Write();
    freeRender();

    Quit::Register();
    Start();
    if (rank == 0) {
      Quit quit;
      quit.Broadcast(true, true);
    }
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
  if (numRanks > 1) {
    sendRays();
    receiveRays();
    return voter->updateState();
  } else {
    return rayQueue.empty();
  }
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

Voter::Voter(int numRanks, int myRank, std::map<int, gvt::render::actor::RayVector> *rayQ)
    : numRanks(numRanks), myRank(myRank), rayQ(rayQ), state(WaitForNoWork), numPendingRays(0), validTimeStamp(0),
      votesAvailable(false), resignGrant(false), numVotesReceived(0), commitCount(0), numPendingVotes(0) {

  pthread_mutex_init(&votingLock, NULL);
  // pthread_mutex_init(&voteWorkBufferLock, NULL);
}

void Voter::addNumPendingRays(int n) {
  pthread_mutex_lock(&votingLock);
  numPendingRays += n;
  pthread_mutex_unlock(&votingLock);
}

void Voter::subtractNumPendingRays(int n) {
  pthread_mutex_lock(&votingLock);
#ifdef DEBUG_RAYTX
  printf("rank %d: Voter::subtractNumPendingRays: numPendingRays(before) %d numPendingRays(after) %d\n", myRank,
         numPendingRays, (numPendingRays - n));
#endif
  numPendingRays -= n;
  assert(numPendingRays >= 0);
  pthread_mutex_unlock(&votingLock);
}

void Voter::bufferVoteWork(VoteWork *work) {
  pthread_mutex_lock(&voteWorkBufferLock);
  voteWorkBuffer.push_back(work); // TODO: avoid resizing
#ifdef DEBUG_RAYTX
  printf("rank %d: Voter::bufferVoteWork received vote request from rank %d voteType %d timeStamp %d\n", myRank,
         work->getSenderRank(), work->getVoteType(), work->getTimeStamp());
#endif
  pthread_mutex_unlock(&voteWorkBufferLock);
}

void Voter::voteForResign(int senderRank, unsigned int timeStamp) {
  int vote = (state == WaitForResign || state == Resigned) ? VoteWork::Commit : VoteWork::Abort;
  VoteWork grant;
  grant.setup(vote, myRank, timeStamp);
  grant.Send(senderRank);
}

void Voter::voteForNoWork(int senderRank, unsigned int timeStamp) {
  int vote = (state != WaitForNoWork) ? VoteWork::Commit : VoteWork::Abort;
  VoteWork grant;
  grant.setup(vote, myRank, timeStamp);
  grant.Send(senderRank);
}
void Voter::applyVoteResult(int voteType, unsigned int timeStamp) {
  pthread_mutex_lock(&votingLock);
  --numPendingVotes;
  assert(numPendingVotes >= 0);
  if (timeStamp == validTimeStamp) {
    ++numVotesReceived;
    commitCount += (voteType == VoteWork::Commit);
    assert(numVotesReceived < numRanks);
    if (numVotesReceived == numRanks - 1) {
      votesAvailable = true;
#ifdef DEBUG_RAYTX
      printf("rank %d: Voter::applyVoteResult: votesAvailable %d, numPendingVotes %d, numVotesReceived %d, commitCount "
             "%d voteType %d\n",
             myRank, votesAvailable, numPendingVotes, numVotesReceived, commitCount, voteType);
#endif
    }
  }
  pthread_mutex_unlock(&votingLock);
}

bool Voter::updateState() {
  pthread_mutex_lock(&votingLock);

  bool hasWork = !(rayQ->empty() && numPendingRays == 0);
  bool allDone = false;

  switch (state) {
  case WaitForNoWork: { // has work
    if (!hasWork) {
      requestForVotes(VoteWork::NoWork, validTimeStamp);
      state = WaitForVotes;
#ifdef DEBUG_RAYTX
      printf("rank %d: WaitForNoWork -> WaitForVotes\n", myRank);
#endif
    }
  } break;
  case WaitForVotes: { // pending votes
    if (hasWork) {
      ++validTimeStamp;
      numVotesReceived = 0;
      commitCount = 0;
      votesAvailable = false;
      state = WaitForNoWork;
#ifdef DEBUG_RAYTX
      printf("rank %d: WaitForVotes -> WaitForNoWork\n", myRank);
#endif
    } else if (votesAvailable) {
      bool commit = checkVotes();

      numVotesReceived = 0;
      commitCount = 0;
      votesAvailable = false;

      if (commit) {
        requestForVotes(VoteWork::Resign, validTimeStamp);
        state = WaitForResign;
#ifdef DEBUG_RAYTX
        printf("rank %d: updateState(): WaitForVotes -> WaitForResign\n", myRank);
#endif
      } else {
#ifdef DEBUG_RAYTX
        printf("rank %d: updateState(): abort message received. retrying requestForVotes.\n", myRank);
#endif
        // TODO: hpark adjust request interval based on workloads?
        requestForVotes(VoteWork::NoWork, validTimeStamp);
      }
    }
  } break;
  case WaitForResign: {
    if (hasWork) {   // TODO: hpark possible? can't just think of this case
      assert(false); // TODO: hpark let's disable this for now
      ++validTimeStamp;
      numVotesReceived = 0;
      commitCount = 0;
      votesAvailable = false;
      state = WaitForNoWork;

    } else if (votesAvailable) {
      bool commit = checkVotes();

      numVotesReceived = 0;
      commitCount = 0;
      votesAvailable = false;

      if (commit) {
        allDone = true;
        state = Resigned;
#ifdef DEBUG_RAYTX
        printf("rank %d: WaitForResign allDone=true\n", myRank);
#endif
      } else {
        requestForVotes(VoteWork::Resign, validTimeStamp);
      }
    }
  } break;
  case Resigned: {
  } break;
  default: { state = WaitForNoWork; } break;
  }
  // vote();
  pthread_mutex_unlock(&votingLock);
  return allDone;
}

void Voter::vote() {
  pthread_mutex_lock(&voteWorkBufferLock);
  for (size_t i = 0; i < voteWorkBuffer.size(); ++i) {
    VoteWork *request = voteWorkBuffer[i];
#ifdef DEBUG_RAYTX
    printf("rank %d: processing vote request type=%d timeStamp=%d\n", myRank, request->getVoteType(),
           request->getTimeStamp());
#endif
    int vote;
    int type = request->getVoteType();
    if (type == VoteWork::NoWork) {
      vote = (state != WaitForNoWork) ? VoteWork::Commit : VoteWork::Abort;
      // } else if (type == VoteWork::Resign) {
      //   vote = (state == WaitForResign) ?  VoteWork::Commit : VoteWork::Abort;
    } else {
      assert(false);
    }
    VoteWork grant;
    grant.setup(vote, myRank, request->getTimeStamp());
    grant.Send(request->getSenderRank());
#ifdef DEBUG_RAYTX
    printf("rank %d: sent vote voteType %d timeStamp %d to rank %d in response to vote Type %d (state %d "
           "numPendingVotes %d)\n",
           myRank, vote, request->getTimeStamp(), request->getSenderRank(), type, state, numPendingVotes);
#endif
    delete request;
  }
  voteWorkBuffer.clear(); // TODO: avoid this
  pthread_mutex_unlock(&voteWorkBufferLock);
}

bool Voter::checkVotes() { return (commitCount == numRanks - 1); }

void Voter::requestForVotes(int voteType, unsigned int timeStamp) {
  numPendingVotes += (numRanks - 1);
  for (int i = 0; i < numRanks; ++i) {
    if (i != myRank) {
      VoteWork work;
      work.setup(voteType, myRank, timeStamp);
      work.Send(i);
#ifdef DEBUG_RAYTX
      printf("rank %d: sent vote request to rank %d voteType %d timeStamp %d\n", myRank, i, voteType, timeStamp);
#endif
    }
  }
}

