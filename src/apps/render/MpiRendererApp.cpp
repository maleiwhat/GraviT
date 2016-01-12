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
/**
 * A simple GraviT application that loads some geometry and renders it
 * using the MpiRenderer class.
 */
#include <gvt/render/RenderContext.h>
#include <gvt/render/Types.h>
#include <vector>
#include <algorithm>
#include <set>
#include <gvt/core/mpi/Wrapper.h>
#include <gvt/core/Math.h>
#include <gvt/render/data/Dataset.h>
#include <gvt/render/data/Domains.h>
#include <gvt/render/Schedulers.h>

#ifdef GVT_RENDER_ADAPTER_EMBREE
#include <gvt/render/adapter/embree/Wrapper.h>
#endif

#ifdef GVT_RENDER_ADAPTER_MANTA
#include <gvt/render/adapter/manta/Wrapper.h>
#endif

#ifdef GVT_RENDER_ADAPTER_OPTIX
#include <gvt/render/adapter/optix/Wrapper.h>
#endif

// #ifdef GVT_USE_MPE
// #include "mpe.h"
// #endif
#include <gvt/render/algorithm/Tracers.h>
#include <gvt/render/data/scene/gvtCamera.h>
#include <gvt/render/data/scene/Image.h>
#include <gvt/render/data/Primitives.h>

#include <boost/range/algorithm.hpp>
#include <gvt/core/mpi/Application.h>
#include <gvt/core/mpi/RenderWork.h>

#include <apps/render/MpiRenderer.h>

#include <iostream>
#include <cstdlib>

using namespace std;
using namespace apps::render;
using namespace gvt::render;
using namespace gvt::core::math;
using namespace gvt::core::mpi;
using namespace gvt::render::data::scene;
using namespace gvt::render::schedule;
using namespace gvt::render::data::primitives;

int main(int argc, char **argv) {

  MPI_Init(&argc, &argv);
  int rank = -1;
  MPI_Comm_rank(MPI_COMM_WORLD, &rank);

  int bunnyCountX = 2;
  int bunnyCountY = 2;
  int bunnyCountZ = 1;

  int schedulerType = gvt::render::scheduler::Domain;
  if (argc > 1) {
    if (*argv[1] == 'i') {
       schedulerType =  gvt::render::scheduler::Image;
    } else if (*argv[1] == 'd') {
       schedulerType =  gvt::render::scheduler::Domain;
    }
    if (argc > 4) {
      bunnyCountX = atoi(argv[2]);
      bunnyCountY = atoi(argv[3]);
      bunnyCountZ = atoi(argv[4]);
    }
  }

  // renderer instance
  MpiRenderer renderer(&argc, &argv);

  // create data node
  Uuid dataNodeId = renderer.createNode("Data", "Data");

  // add a mesh
  Uuid bunnyMeshNodeId
      = renderer.addMesh(dataNodeId, "bunny_mesh",
                         "../data/geom/bunny.obj");

  // create instances node
  Uuid instancesNodeId = renderer.createNode("Instances", "Instances");

  // add instances
  Box3D bunnyBounds = renderer.getMeshBounds(bunnyMeshNodeId);
  Vector3f extent = bunnyBounds.extent();

  const float gapX = extent[0] * 0.2f;
  const float gapY = extent[1] * 0.2f;
  const float gapZ = extent[2] * 0.2f;

  Vector3f minPos((extent[0] + gapX) * bunnyCountX * -0.5f,
                  (extent[1] + gapY) * bunnyCountY * -0.5f,
                  (extent[2] + gapZ) * bunnyCountZ * -0.5f);

  int instanceId = 0;
  for (int z=0; z<bunnyCountZ; ++z) {
    for (int y=0; y<bunnyCountY; ++y) {
      for (int x=0; x<bunnyCountX; ++x) {
        // int instanceId = y * 2 + x;
        // m: transform matrix
        auto m = new gvt::core::math::AffineTransformMatrix<float>(true);
        *m = *m * gvt::core::math::AffineTransformMatrix<float>::
            createTranslation(minPos[0] + x * (extent[0] + gapX),
                              minPos[1] + y * (extent[1] + gapY),
                              minPos[2] + z * (extent[2] + gapZ));
        *m = *m * gvt::core::math::AffineTransformMatrix<float>::
            createScale(1.0f, 1.0f, 1.0f);
        Uuid bunnyId =
            renderer.addInstance(instancesNodeId, bunnyMeshNodeId,
                                 instanceId++, "bunny", m);
      }
    }
  }

  // create lights node
  Uuid lightsNodeId = renderer.createNode("Lights", "Lights");

  // add lights
  Vector4f lightPosition(0.0, 0.1, 0.5, 0.0);
  Vector4f lightColor(1.0, 1.0, 1.0, 0.0);
  Uuid lightNodeId = renderer.addPointLight(lightsNodeId, "point_light",
                                            lightPosition, lightColor);

  // create camera node
  Point4f eye(0.0, 0.5, 1.2, 1.0);
  Point4f focus(0.0, 0.0, 0.0, 1.0);
  Vector4f upVector(0.0, 1.0, 0.0, 0.0);
  float fov = (45.0 * M_PI / 180.0);
  // const unsigned int width = 1920;
  // const unsigned int height = 1080;
  const unsigned int width = 640;
  const unsigned int height = 480;
  Uuid cameraNodeId =
      renderer.createCameraNode(eye, focus, upVector, fov, width, height);

  // create film node
  Uuid filmNodeId = renderer.createFilmNode(width, height, "bunnies");

  // create the scheduler node
  // valid schedulers = {Image, Domain}
  // valid adapters = {Embree, Manta, Optix}
#ifdef GVT_RENDER_ADAPTER_EMBREE
  int adapterType = gvt::render::adapter::Embree;
#elif GVT_RENDER_ADAPTER_MANTA
  int adapterType = gvt::render::adapter::Manta;
#elif GVT_RENDER_ADAPTER_OPTIX
  int adapterType = gvt::render::adapter::Optix;
#else
  GVT_DEBUG(DBG_ALWAYS, "ERROR: missing valid adapter");
#endif

  adapterType = gvt::render::adapter::Embree;
  Uuid scheduleNodeId =
      renderer.createScheduleNode(schedulerType, adapterType);

  renderer.render();

  if (MPI::COMM_WORLD.Get_size() > 1)
    MPI_Finalize();
}
