/* =======================================================================================
   This file is released as part of GraviT - scalable, platform independent ray tracing
   tacc.github.io/GraviT

   Copyright 2013-2017 Texas Advanced Computing Center, The University of Texas at Austin
   All rights reserved.

   Licensed under the BSD 3-Clause License, (the "License"); you may not use this file
   except in compliance with the License.
   A copy of the License is included with this software in the file LICENSE.
   If your copy does not contain the License, you may obtain a copy of the License at:

       http://opensource.org/licenses/BSD-3-Clause

   Unless required by applicable law or agreed to in writing, software distributed under
   the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
   KIND, either express or implied.
   See the License for the specific language governing permissions and limitations under
   limitations under the License.

   GraviT is funded in part by the US National Science Foundation under awards ACI-1339863,
   ACI-1339881 and ACI-1339840
   ======================================================================================= */

/* 
 * This utility measures execution cost for TBB-based parallel-for ray intersection versus 
 * single ray intersection.
 * 
 * pnav
 */



#include "gvt/core/Math.h"
#include "gvt/core/utils/timer.h"

#include "gvt/render/data/primitives/BBox.h"

#include <iostream>
#include <random>
#include <string>

bool verbose = false;

void intersect_single_loop(int rays, int bbs, glm::vec3* o, glm::vec3* dir, gvt::render::data::primitives::Box3D* boxes)
{
   std::cout << "intersecting single loop" << std::endl;

}

void intersect_tbb_loop(int rays, int bbs, glm::vec3* o, glm::vec3* dir, gvt::render::data::primitives::Box3D* boxes)
{
   std::cout << "intersecting TBB parallel for loop" << std::endl;

}


void benchmark_rays(int rays, int bbs) 
{
   std::cout << "Allocating " << rays << " rays" << std::endl;
   glm::vec3* o = new glm::vec3[rays];
   glm::vec3* d = new glm::vec3[rays];
   std::default_random_engine generator;
   std::uniform_real_distribution<float> dir_distribution(0,1);
   auto dir = std::bind(dir_distribution, generator);

   for (int i = 0; i < rays; ++i)
   {
      o[i].x = o[i].y = o[i].z = 0;
      d[i].x = dir(); d[i].y = dir(); d[i].z = dir();
      if (verbose)
         std::cout << "\tray " << i << " o:" << o[i] << " d:" << d[i] << std::endl;
   }

   std::cout << "Allocating " << bbs << " bounding boxes" << std::endl;
   gvt::render::data::primitives::Box3D* boxes = new gvt::render::data::primitives::Box3D[bbs];
   std::uniform_int_distribution<int> box_distribution(1,100);
   auto box = std::bind(box_distribution, generator);

   for (int i = 0; i < bbs; ++i)
   {
      glm::vec3 min(box(),box(),box()), max(box(),box(),box());
      boxes[i] = gvt::render::data::primitives::Box3D(glm::vec3(box(),box(),box()), glm::vec3(box(),box(),box()));
      if (verbose)
         std::cout << "\tbox " << i << " " << boxes[i] << std::endl;
   }

   intersect_single_loop(rays,bbs,o,d,boxes);
   intersect_tbb_loop(rays,bbs,o,d,boxes);

   std::cout << "Cleaning up" << std::endl;

   delete [] o;
   delete [] d;
   delete [] boxes;
}


int main(int argc, char** argv)
{
   int rays=10, bbs=1;
   for (int i=1; i < argc; ++i)
   {
      std::string s(argv[i]);
      if (s == "-v" || s == "--verbose") verbose = true;
      else if ((s == "-r" || s == "--rays") && (i+1) < argc) rays = std::stoi(argv[++i]);
      else if ((s == "-b" || s == "--boxes") && (i+1) < argc) bbs = std::stoi(argv[++i]);
      else if (s == "-h" || s == "--help")
      {
         std::cerr << "usage: " << argv[0] << " <args>" << std::endl;
         std::cerr << "    -h, --help      - this message" << std::endl;
         std::cerr << "    -r n, --rays n  - number of rays to intersect" << std::endl;
         std::cerr << "    -b n, --boxes n - number of bounding boxes to intersect" << std::endl;
         std::cerr << "    -v, --verbose   - verbose execution" << std::endl;
         return -1;
      }
      else 
      {
         std::cerr << "unrecognized option: " << argv[i] << std::endl;
         argv[i--] = "-h";
      }
   }

   benchmark_rays(rays, bbs);
   return 0;
}


