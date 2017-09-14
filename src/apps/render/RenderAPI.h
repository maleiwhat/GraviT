/* =======================================================================================
   This file is released as part of GraviT - scalable, platform independent ray tracing
   tacc.github.io/GraviT

   Copyright 2013-2015 Texas Advanced Computing Center, The University of Texas at Austin
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
/**
 * Provides a simple API for external applications to easily feed data and render images.
 *
*/

#include <stdio.h>
#include <vector>
#include <limits>

class VisitAdapter {

public:
  enum Scheduler {image, domain};
  enum LightType {camera, object};

  struct RayTraceProperties {
    int maxDepth;
    int raySamples;
    double windowJitterSize;
    unsigned char backgroundColor[3];
  };


  VisitAdapter () {
    //If mpi init has not being called, call it
    initMPI();

    gravitProgramConfig.dataBoundingBox[0] = std::numeric_limits<double>::max();
    gravitProgramConfig.dataBoundingBox[1] = std::numeric_limits<double>::max();
    gravitProgramConfig.dataBoundingBox[2] = std::numeric_limits<double>::max();

    gravitProgramConfig.dataBoundingBox[3] = std::numeric_limits<double>::min();
    gravitProgramConfig.dataBoundingBox[4] = std::numeric_limits<double>::min();
    gravitProgramConfig.dataBoundingBox[5] = std::numeric_limits<double>::min();
  };

  void setVisitProcessBlockFunc(void * obj, int(*loadBlock)(void *, int, double **, int &, int **, int & )) {
    loadBlockObj = obj;
    loadBlockFunc = loadBlock;
  }

  void resetMeshAndInstance();

  void setData(double * points, int numPoints, int * edges, int numEdges, int material, double * materialProp);

  /*
    ImageTracer Only: place an box mesh to indicate data is here, when the image tracer hits the domain
    the loadblockFunc will be called and the data will be requested.
  */
  void setBoundingBoxHolder(double * points, int numPoints, int * edges, int numEdges,
                            int material, double * materialProp);

  //Tracer Configuration Methods

  //Specify the number of lights with numberOfLights and then pass in an array of their properties
  // for each of light type, direction, color and intensity.
  void setLight(int numberOfLights, LightType * lighttypes, double * lightDirection, unsigned char * color,
                double * lightIntensity);

  //0 based
  double * getLightWorldCoordinates(int lightNumber) {
    int totalLights = gravitProgramConfig.lightWorldCoordinates.size();
    if(lightNumber < totalLights / 3) {
      return &gravitProgramConfig.lightWorldCoordinates[lightNumber * 3];
    }
    return NULL;
  }

  void setRayTraceProperties(RayTraceProperties properties);

  void setTraceMode(Scheduler mode) {
    gravitProgramConfig.traceScheduler = mode;
  }

  void changeMaterial(int meshId, int material, double * materialProp);

  void setCamera(int * imageSize,  double * focalPoint, double * upVector,
                 double * viewDirection, double zoom, double fov);

  double * getCameraWorldCoordinates() {
    return gravitProgramConfig.cameraWorldCoordinates;
  };

  void draw(unsigned char * outputImage);

protected:
  struct GravitProgramConfig {
    // Camera
    double focalPoint[3];
    double upVector[3];
    double view_direction[3];
    double fov;
    double zoom;
    double cameraWorldCoordinates[3];
    int filmSize[2];

    //Tracing
    int maxDepth;
    int raySamples;
    double windowJitterSize;
    unsigned char backgroundColor[3];

    //Others
    double dataBoundingBox[6];
    std::vector<double> lightWorldCoordinates; 

    Scheduler traceScheduler;
  };

  void initMPI();
  struct GravitProgramConfig gravitProgramConfig;
  int (*loadBlockFunc)(void *, int, double **, int &, int **, int & );
  void * loadBlockObj;
};
