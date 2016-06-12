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

#include <sys/stat.h>
#include <tbb/task_scheduler_init.h>
#include <cstdlib>
#include <iostream>
#include <thread>

#include "gvt/core/Math.h"

#include "gvt/render/unit/CommonWorks.h"
#include "gvt/render/unit/DomainTracer.h"
#include "gvt/render/unit/DomainWorks.h"
#include "gvt/render/unit/TestTracer.h"
#include "gvt/render/unit/Worker.h"

#ifndef MAX
#define MAX(a, b) ((a > b) ? (a) : (b))
#endif

#ifndef MIN
#define MIN(a, b) ((a < b) ? (a) : (b))
#endif

namespace apps {
namespace render {
namespace mpi {

struct Options {
  enum SchedulerType {
    PING_TEST = 0,
    ASYNC_IMAGE,
    ASYNC_DOMAIN,
    SYNC_IMAGE,
    SYNC_DOMAIN,
    NUM_TRACERS
  };

  enum AdapterType { EMBREE, MANTA, OPTIX };

  int tracerType = ASYNC_DOMAIN;
  int adapterType = EMBREE;
  int width = 1920;
  int height = 1080;
  bool obj = false;
  int instanceCountX = 1;
  int instanceCountY = 1;
  int instanceCountZ = 1;
  int numFrames = 1;
  int numTbbThreads;
  std::string infile;
};

void PrintUsage(const char* argv) {
  printf(
      "Usage : %s [-h] [-i <infile>] [-a <adapter>] [-n <x y z>] [-p] [-s "
      "<scheduler>] [-W <image_width>] [-H "
      "<image_height>] [-N <num_frames>] [-t <num_tbb_threads>]\n",
      argv);
  printf("  -h, --help\n");
  printf(
      "  -i, --infile <infile> (default: ../data/geom/bunny.obj for obj and "
      "./EnzoPlyData/Enzo8 for ply)\n");
  printf("  -a, --adapter <embree | manta | optix> (default: embree)\n");
  printf("  -t, --tracer <0-3> (default: 2)\n");
  printf(
      "      0: PING_TEST, 0: ASYNC_IMAGE, 1: ASYNC_DOMAIN, 2: SYNC_IMAGE, 3: "
      "SYNC_DOMAIN\n");
  printf(
      "  -n, --num-instances <x, y, z> specify the number of instances in each "
      "direction (default: 1 1 1). "
      "effective only with obj.\n");
  printf("  --obj use obj models (not ply models)\n");
  printf("  -W, --width <image_width> (default: 1920)\n");
  printf("  -H, --height <image_height> (default: 1080)\n");
  printf("  -N, --num-frames <num_frames> (default: 1)\n");
  printf("  --tbb <num_tbb_threads>\n");
  printf(
      "      (default: # cores for sync. schedulers or # cores - 2 for async. "
      "schedulers)\n");
}

void ParseCommandLine(int argc, char** argv, Options* options) {
  options->numTbbThreads = -1;
  for (int i = 1; i < argc; ++i) {
    if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
      PrintUsage(argv[0]);
      exit(1);
    } else if (strcmp(argv[i], "-i") == 0 || strcmp(argv[i], "--infile") == 0) {
      options->infile = argv[++i];
      struct stat buf;
      if (stat(options->infile.c_str(), &buf) != 0) {
        printf("error: file not found. %s\n", options->infile.c_str());
        exit(1);
      }
    } else if (strcmp(argv[i], "-a") == 0 ||
               strcmp(argv[i], "--adapter") == 0) {
      ++i;
      if (strcmp(argv[i], "embree") == 0) {
        options->adapterType = Options::EMBREE;
      } else if (strcmp(argv[i], "manta") == 0) {
        options->adapterType = Options::MANTA;
      } else if (strcmp(argv[i], "optix") == 0) {
        options->adapterType = Options::OPTIX;
      } else {
        printf("error: %s not defined\n", argv[i]);
        exit(1);
      }
    } else if (strcmp(argv[i], "-t") == 0 || strcmp(argv[i], "--tracer") == 0) {
      options->tracerType = atoi(argv[++i]);
      if (options->tracerType < 0 ||
          options->tracerType >= Options::NUM_TRACERS) {
        printf("error: %s not defined\n", argv[i]);
        exit(1);
      }
    } else if (strcmp(argv[i], "-n") == 0 ||
               strcmp(argv[i], "--num-instances") == 0) {
      options->instanceCountX = atoi(argv[++i]);
      options->instanceCountY = atoi(argv[++i]);
      options->instanceCountZ = atoi(argv[++i]);
      // } else if (strcmp(argv[i], "-p") == 0 || strcmp(argv[i], "--ply") == 0)
      // {
      //   options->ply = true;
    } else if (strcmp(argv[i], "--obj") == 0) {
      options->obj = true;
    } else if (strcmp(argv[i], "-W") == 0 || strcmp(argv[i], "--width") == 0) {
      options->width = atoi(argv[++i]);
    } else if (strcmp(argv[i], "-H") == 0 || strcmp(argv[i], "--height") == 0) {
      options->height = atoi(argv[++i]);
    } else if (strcmp(argv[i], "-N") == 0 ||
               strcmp(argv[i], "--num-frames") == 0) {
      options->numFrames = atoi(argv[++i]);
    } else if (strcmp(argv[i], "--tbb") == 0) {
      options->numTbbThreads = atoi(argv[++i]);
    } else {
      printf("error: %s not defined\n", argv[i]);
      exit(1);
    }
  }
  if (options->numTbbThreads <= 0) {
    if (options->tracerType == 2 || options->tracerType == 3) {
      options->numTbbThreads = MAX(1, std::thread::hardware_concurrency());
    } else {
      options->numTbbThreads = MAX(1, std::thread::hardware_concurrency() - 2);
    }
  }
  if (options->infile.empty()) {
    if (options->obj) {
      options->infile = std::string("../data/geom/bunny.obj");
    } else {
      options->infile = std::string("./EnzoPlyData/Enzo8/");
    }
  }
}

}  // namespace mpi
}  // namespace render
}  // namespace apps

using namespace apps::render::mpi;
using namespace gvt::render::unit;

int main(int argc, char** argv) {
  Options options;
  ParseCommandLine(argc, argv, &options);

  // create a ray tracer
  RayTracer* tracer = NULL;
  if (options.tracerType == Options::PING_TEST) {
    tracer = new PingTracer;
  } else {
    std::cout << "error found unsupported tracer type " << options.tracerType
              << "\n";
    exit(1);
  }

  // create a worker
  Worker worker(tracer);

  // register works
  Command::Register(&worker);
  PingTest::Register(&worker);
  RemoteRays::Register(&worker);

#ifndef NDEBUG
  std::cout << "worker start\n";
#endif
  // start the worker
  worker.Start(argc, argv);

#ifndef NDEBUG
  std::cout << "worker done\n";
#endif
  // start the worker

  // delete the tracer
  delete tracer;

  // renderer.parseCommandLine(argc, argv);
  // renderer.createDatabase();
  // renderer.render();
}

