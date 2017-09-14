#include <stdio.h>
#include <fstream>
#include <iostream>
#include <sstream>
#include <mpi.h>
#include <cassert>
#include <boost/version.hpp>

#include <gvt/core/Math.h>
#include <gvt/core/context/Variant.h>
#include <apps/render/RenderAPI.h>
#include <apps/test/testingHelper.h>
#include <apps/render/ParseCommandLine.h>

#define IMAGE_HEIGHT 600
#define IMAGE_WIDTH 600

using namespace std;

void writeImage(unsigned char * rgb, int width, int height, string filename) {
  std::stringstream header;
  header << "P6" << std::endl;
  header << width << " " << height << std::endl;
  header << "255" << std::endl;

  std::fstream file;
  string ext = ".ppm";
  file.open((filename + ext).c_str(), std::fstream::out | std::fstream::trunc | std::fstream::binary);
  file << header.str();

  // reverse row order so image is correctly oriented
  for (int j = height - 1; j >= 0; j--) {
    int offset = j * width;
    for (int i = 0; i < width; ++i) {
      int index = 3 * (offset + i);
      file << rgb[index + 0] << rgb[index + 1] << rgb[index + 2];
    }
  }

  file.close();
}

void generateGeometry(VisitAdapter & adapter, int rank,  double modifier) {

  //draw a simple triangle
  double points[9];
  int edges[3];

  if (rank == 1) {
    //second point
    points[0] = 1.0 * modifier;
    points[1] = 1.0 * modifier;
    points[2] = 1;

    //thrid point
    points[3] = -1.0 * modifier;
    points[4] = 1.0 * modifier;
    points[5] = 1;

    //fourth point
    points[6] = -1.0 * modifier;
    points[7] = -1.0 * modifier;
    points[8] = -0.5;

    edges[0] = 1;
    edges[1] = 2;
    edges[2] = 3;

  } else { //second rank
    //first point
    points[0] = 1.0 * modifier;
    points[1] = -1.0 * modifier;
    points[2] = 1;

    //second point
    points[3] = 1.0 * modifier;
    points[4] = 1.0 * modifier;
    points[5] = 1;

    //fourth point
    points[6] = -1.0 * modifier;
    points[7] = -1.0 * modifier;
    points[8] = -0.5;

    edges[0] = 1;
    edges[1] = 2;
    edges[2] = 3;
  }

  //set material diffused
  double kd[3] = {0.5, 0.5, 0.5};
  int materialType = 0; //lambert

  adapter.setData(points, 3,  edges,  1, materialType, kd);
}

void placeCamera(VisitAdapter & adapter, double * focus) {
  int imageSize[2] = {IMAGE_WIDTH, IMAGE_HEIGHT};
  double focalPoint[3] = {focus[0], focus[1], focus[2]};

  double upVector[3] = {0, 1, 0};

  double viewDirection[3] = {0, 0, 1};
  double zoom = 10;
  double fov = 30;

  adapter.setCamera(imageSize, focalPoint, upVector, viewDirection,
                    zoom, fov);
}

void placeCamera(VisitAdapter & adapter) {
  double focus[3] = {0, 0, 0};
  placeCamera(adapter, focus);
}

void placeLight(VisitAdapter & adapter) {
  int numLights = 1;
  VisitAdapter::LightType lt = VisitAdapter::camera; //light always focuses at the middle of view

  double lightDirection[3] = {0, 0, -100};
  unsigned char color[3] = {255, 255, 255};

  double lightIntensity = 30.0;
  adapter.setLight(numLights, &lt, lightDirection, color,
                   &lightIntensity);
}

void drawImage(VisitAdapter & adapter, string fileName, int rank) {
  //create a image
  unsigned char image[IMAGE_WIDTH * IMAGE_HEIGHT * 3];
  adapter.draw(image);
  if(rank == 0) {
    writeImage(image, IMAGE_WIDTH, IMAGE_HEIGHT, fileName);
  }
}

void setRayTraceProperties(VisitAdapter & adapter) {
  //Trace using image tracer
  VisitAdapter::Scheduler traceSchedule = VisitAdapter::domain;
  adapter.setTraceMode(traceSchedule);

  VisitAdapter::RayTraceProperties traceProp;
  traceProp.maxDepth = 1;
  traceProp.raySamples = 1;
  traceProp.windowJitterSize = 0.d;

  adapter.setRayTraceProperties(traceProp);
}

void resetAdapter(VisitAdapter & adapter) {
  adapter.resetMeshAndInstance();
}

int main(int argc, char * argv[]) {
  /*
    This test is designed to be run with 2 ranks
  */
  ParseCommandLine cmd("renderAPITest");
  cmd.addoption("outputDirectory", ParseCommandLine::PATH, "Output Image Directory Path", 1);
  cmd.parse(argc, argv);

  string outputDirectory = "";
  if (cmd.isSet("outputDirectory")) {
    outputDirectory = cmd.getValue<std::string>("outputDirectory")[0];
    outputDirectory += "/"; 
  }

  MPI_Init(&argc, &argv);

  int rank = -1;
  int size = -1;

  MPI_Comm_rank(MPI_COMM_WORLD, &rank);
  MPI_Comm_size (MPI_COMM_WORLD, &size);

  if (size != 2 ) {
    raiseTestingError("Render API testing program was not launched with two mpi ranks");
  }

  VisitAdapter adapter;

  //scene configuration
  double firstModifier = 2.0;
  generateGeometry(adapter, rank, firstModifier);
  placeCamera(adapter);

  // verify camera position
  double * cameraPosition = adapter.getCameraWorldCoordinates();

  assert(cameraPosition[0] == 0 &&
         cameraPosition[1] == 0 &&
         cameraPosition[2] == 14.0);

  placeLight(adapter);

  //verify light position
  double * lightPosition = adapter.getLightWorldCoordinates(0); 
  assert(lightPosition[0] == 0 &&
         lightPosition[1] == 0 &&
         isCloseTo(lightPosition[2], 7.60303, 0.001));

  //tracer configuration -- use domain tracer
  setRayTraceProperties(adapter);

  //draw initial Image
  drawImage(adapter, outputDirectory + "firstImage", rank);

  /* ------------------------------------------*/
  /*--- Reset and draw second image -----------*/

  //set data for second image
  MPI_Barrier(MPI_COMM_WORLD);
  resetAdapter(adapter);

  MPI_Barrier(MPI_COMM_WORLD);
  double secondModifier = 1.0;
  generateGeometry(adapter, rank, secondModifier);
  placeCamera(adapter);
  placeLight(adapter);

  //draw second image
  drawImage(adapter, outputDirectory + "secondImage",rank);

  /* ------------------------------------------*/
  /*--- Modify Camera Position and draw third image */
  
  double newFocus[3] = {5, 0, 0};
  placeCamera(adapter, newFocus);

  //draw third image
  drawImage(adapter, outputDirectory + "thirdImage", rank);
  //return
  if (MPI::COMM_WORLD.Get_size() > 1) MPI_Finalize();
  return 0;
}
