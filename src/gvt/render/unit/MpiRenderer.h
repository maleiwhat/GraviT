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
//
// MpiRenderer.h
//

#ifndef GVT_RENDER_UNIT_MPI_RENDERER_H
#define GVT_RENDER_UNIT_MPI_RENDERER_H

#include "gvt/core/mpi/Application.h"
#include "gvt/core/DatabaseNode.h"
#include "gvt/core/math/Vector.h"
#include "gvt/core/Types.h"

#include "gvt/render/Types.h"
#include "gvt/render/data/primitives/BBox.h"

#include <vector>
#include <map>
#include <tbb/mutex.h>
#include <pthread.h>

using namespace gvt::core::mpi;

namespace gvt {
namespace render {
class RenderContext;
}
}

namespace gvt {
namespace render {
namespace data {
namespace scene {
class gvtPerspectiveCamera;
class Image;
}
}
}
}

namespace gvt {
namespace render {
namespace data {
namespace accel {
class AbstractAccel;
}
}
}
}

namespace gvt {
namespace render {
namespace unit {
namespace rank {
enum RankType { Server = 0, Display = 1, FirstWorker };
}

class TileLoadBalancer;
class RayTransferWork;
class VoteWork;

struct MpiRendererOptions {
  virtual ~MpiRendererOptions() {}
  int schedulerType = gvt::render::scheduler::Domain;
  int adapterType = gvt::render::adapter::Embree;
  int width = 1280;
  int height = 720;
  bool asyncMpi = true;
  bool ply = false;
  int instanceCountX = 1;
  int instanceCountY = 1;
  int instanceCountZ = 1;
};

class Voter;

class MpiRenderer : public Application {
public:
  MpiRenderer(int *argc, char ***argv);
  virtual ~MpiRenderer();

  // for configuring database
  void printUsage(const char *argv);
  void parseCommandLine(int argc, char **argv);
  void createDatabase();

  // helper APIs for creating database
  bool isNodeTypeReserved(const std::string &type);

  gvt::core::DBNodeH getNode(const gvt::core::Uuid &id);

  gvt::core::Uuid createNode(const std::string &type, const std::string &name);

  gvt::core::Uuid addMesh(const gvt::core::Uuid &parentNodeId, const std::string &meshName,
                          const std::string &objFilename);

  gvt::render::data::primitives::Box3D getMeshBounds(const gvt::core::Uuid &id);
  gvt::render::data::primitives::Box3D getMeshBounds(const std::string &objFilename);

  gvt::core::Uuid addInstance(const gvt::core::Uuid &parentNodeId, const gvt::core::Uuid &meshId, int instanceId,
                              const std::string &instanceName,
                              gvt::core::math::AffineTransformMatrix<float> *transform);

  gvt::core::Uuid addPointLight(const gvt::core::Uuid &parentNodeId, const std::string &lightName,
                                const gvt::core::math::Vector4f &position, const gvt::core::math::Vector4f &color);

  gvt::core::Uuid createCameraNode(const gvt::core::math::Point4f &eye, const gvt::core::math::Point4f &focus,
                                   const gvt::core::math::Vector4f &upVector, float fov, unsigned int width,
                                   unsigned int height);

  gvt::core::Uuid createFilmNode(int width, int height, const std::string &sceneName);

  gvt::core::Uuid createScheduleNode(int schedulerType, int adapterType);
  void render();

private:
  void run();
  void initServer();
  void setupRender();
  void freeRender();
  void initInstanceRankMap();
  void makeObjDatabase();
  void makePlyDatabase();

public:
  // database, context, and domain mapping
  gvt::render::RenderContext *getRenderContext() { return renderContext; }
  gvt::core::Vector<gvt::core::DBNodeH> &getInstanceNodes() { return instanceNodes; }
  std::size_t getInstanceNodesSize() const { return instanceNodes.size(); }
  gvt::core::DBNodeH getMeshNode(int domainId) { return instanceNodes[domainId]["meshRef"].deRef(); }
  gvt::core::DBNodeH getInstanceNode(int domainId) { return instanceNodes[domainId]; }
  int getInstanceOwner(int domainId) { return instanceRankMap[instanceNodes[domainId].UUID()]; }

private:
  MpiRendererOptions options;
  gvt::render::RenderContext *renderContext;
  gvt::core::Vector<gvt::core::DBNodeH> instanceNodes;
  gvt::core::DBNodeH root;
  std::map<gvt::core::Uuid, int> instanceRankMap;

public:
  // camera, load balancer, world bvh
  const gvt::render::data::scene::gvtPerspectiveCamera *getCamera() const { return camera; }
  TileLoadBalancer *getTileLoadBalancer() { return tileLoadBalancer; }
  gvt::render::data::accel::AbstractAccel *getAcceleration() { return acceleration; }

private:
  gvt::render::data::scene::gvtPerspectiveCamera *camera;
  TileLoadBalancer *tileLoadBalancer;
  gvt::render::data::accel::AbstractAccel *acceleration;

public:
  // ray queue
  std::map<int, gvt::render::actor::RayVector> *getRayQueue() { return &rayQueue; }
  std::map<int, gvt::render::actor::RayVector> *getIncomingRayQueue() { return &incomingRayQueue; }
  tbb::mutex *getRayQueueMutex() { return rayQueueMutex; }
  tbb::mutex *getIncomingRayQueueMutex() { return &incomingRayQueueMutex; }
  bool isRayQueueEmpty() const { return rayQueue.empty(); }
  bool isIncomingRayQueueEmpty() const { return incomingRayQueue.empty(); }
  pthread_mutex_t *getRayTransferMutex() { return &rayTransferMutex; }

private:
  std::map<int, gvt::render::actor::RayVector> rayQueue;
  std::map<int, gvt::render::actor::RayVector> incomingRayQueue;
  tbb::mutex *rayQueueMutex;
  tbb::mutex incomingRayQueueMutex;
  pthread_mutex_t rayTransferMutex; // TODO: too coarse grained, use rayQueueMutex instead

public:
  // image
  tbb::mutex *getColorBufMutex() { return colorBufMutex; }
  gvt::render::data::scene::Image *getImage() { return image; }
  std::vector<GVT_COLOR_ACCUM> *getFramebuffer() { return &framebuffer; }
  void aggregatePixel(int pixelId, const GVT_COLOR_ACCUM &color);
  void updatePixel(int pixelId, const GVT_COLOR_ACCUM &color) { framebuffer[pixelId] = color; }
  int decrementPendingPixelCount(int amount) {
    pendingPixelCount -= amount;
    return pendingPixelCount;
  }
  int getImageWidth() const { return imageWidth; }
  int getImageHeight() const { return imageHeight; }

private:
  tbb::mutex *colorBufMutex;
  gvt::render::data::scene::Image *image;
  std::vector<GVT_COLOR_ACCUM> framebuffer;
  int pendingPixelCount;
  int imageWidth;
  int imageHeight;

// ray transfer
public:
  bool transferRays();
  void bufferRayTransferWork(RayTransferWork* work);
  void bufferVoteWork(VoteWork* work);
  void voteForResign(int senderRank, unsigned int timeStamp);
  void voteForNoWork(int senderRank, unsigned int timeStamp);
  void applyRayTransferResult(int numRays);
  void applyVoteResult(int voteType, unsigned int timeStamp);
  void copyIncomingRays(int instanceId, const gvt::render::actor::RayVector *incomingRays);

private:
  int myRank;
  int numRanks;

  void sendRays();
  void receiveRays();

  Voter* voter;

  // Warning: delete these dynamicaly allocated pointers upon copying all rays
  // TODO: minimize resizing
  std::vector<RayTransferWork*> rayTransferBuffer;
  pthread_mutex_t rayTransferBufferLock;

private:
  friend class PixelGatherWork;
  friend class PixelWork;
  friend class RequestWork;

  bool imageReady;
  pthread_mutex_t imageReadyLock;
  pthread_cond_t imageReadyCond;

  bool serverReady;
  pthread_mutex_t serverReadyLock;
  pthread_cond_t serverReadyCond;
};

class Voter {
public:
  Voter(int numRanks, int myRank, std::map<int, gvt::render::actor::RayVector> *rayQ);

  bool updateState();
  void addNumPendingRays(int n);
  void subtractNumPendingRays(int n);

  void applyVoteResult(int voteType, unsigned int timeStamp);
  void bufferVoteWork(VoteWork* work);
  void voteForResign(int senderRank, unsigned int timeStamp);
  void voteForNoWork(int senderRank, unsigned int timeStamp);

private:
  enum State { WaitForNoWork, WaitForVotes, WaitForResign, Resigned };

  void vote();
  bool checkVotes();
  void requestForVotes(int voteType, unsigned int timeStamp);

  const int numRanks;
  const int myRank;
  const std::map<int, gvt::render::actor::RayVector> *rayQ;

  int state;

  pthread_mutex_t votingLock;
  int numPendingRays;
  unsigned int validTimeStamp;
  bool votesAvailable;
  bool resignGrant;

  int numVotesReceived;
  int commitCount;

  int numPendingVotes;

  std::vector<VoteWork*> voteWorkBuffer;
  pthread_mutex_t voteWorkBufferLock;
};

}
}
}

#endif
