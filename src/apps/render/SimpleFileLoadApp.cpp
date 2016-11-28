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

   GraviT is funded in part by the US National Science Foundation under awards
   ACI-1339863,
   ACI-1339881 and ACI-1339840
   =======================================================================================
   */
//
// Simple gravit application.
// Load some geometry and render it.
//
#include <algorithm>
#include <gvt/core/Math.h>
#include <gvt/core/mpi/Wrapper.h>
#include <gvt/render/Context.h>
#include <gvt/render/Schedulers.h>
#include <gvt/render/Types.h>
#include <set>
#include <vector>

#include <tbb/task_scheduler_init.h>
#include <thread>

#include <gvt/core/Context.h>
#include <gvt/core/comm/comm.h>
#include <gvt/render/Context.h>

#include <gvt/render/composite/IceTComposite.h>
#include <gvt/render/composite/ImageComposite.h>
#include <gvt/render/tracer/Domain/DomainTracer.h>
#include <gvt/render/tracer/Image/ImageTracer.h>


#ifdef GVT_RENDER_ADAPTER_EMBREE
#include <gvt/render/adapter/embree/Wrapper.h>
#endif

#ifdef GVT_RENDER_ADAPTER_MANTA
#include <gvt/render/adapter/manta/Wrapper.h>
#endif

#ifdef GVT_RENDER_ADAPTER_OPTIX
#include <gvt/render/adapter/optix/Wrapper.h>
#endif

#include <gvt/core/data/primitives/BBox.h>
#include <gvt/render/algorithm/Tracers.h>
#include <gvt/render/data/Primitives.h>
#include <gvt/render/data/reader/ObjReader.h>
#include <gvt/render/data/scene/Image.h>
#include <gvt/render/data/scene/gvtCamera.h>

#include <boost/range/algorithm.hpp>

#include <iostream>

#include "ParseCommandLine.h"

using namespace std;
using namespace gvt::render;

using namespace gvt::core::mpi;
using namespace gvt::render::data::scene;
using namespace gvt::render::schedule;
using namespace gvt::render::data::primitives;

int setContext(int argc, char **argv) {
  ParseCommandLine cmd("gvtFileLoad");
  cmd.addoption("obj", ParseCommandLine::PATH, "Location of Obj object", 1);
  cmd.addoption("wsize", ParseCommandLine::INT, "Window size", 2);
  cmd.addoption("eye", ParseCommandLine::FLOAT, "Camera position", 3);
  cmd.addoption("look", ParseCommandLine::FLOAT, "Camera look at", 3);
  cmd.addoption("image", ParseCommandLine::NONE, "Use embeded scene", 0);
  cmd.addoption("domain", ParseCommandLine::NONE, "Use embeded scene", 0);
  cmd.addoption("threads", ParseCommandLine::INT,
                "Number of threads to use (default number cores + ht)", 1);
  cmd.addoption("output", ParseCommandLine::PATH, "Output Image Path", 1);
  cmd.addconflict("image", "domain");
  cmd.parse(argc, argv);
  if (!cmd.isSet("threads")) {
    tbb::task_scheduler_init init(std::thread::hardware_concurrency());
  } else {
    tbb::task_scheduler_init init(cmd.get<int>("threads"));
  }

  std::shared_ptr<gvt::comm::communicator> comm = gvt::comm::communicator::singleton();


  gvt::render::RenderContext *cntxt = gvt::render::RenderContext::instance();
  if (cntxt == NULL) {
    std::cout << "Something went wrong initializing the context" << std::endl;
    exit(0);
  }

  gvt::core::DBNodeH root = cntxt->getRootNode();

  if (comm->id() == 0) {
    gvt::core::DBNodeH dataNodes =
        cntxt->addToSync(cntxt->createNodeFromType("Data", "Data", root.UUID()));
    cntxt->addToSync(cntxt->createNodeFromType("Mesh", "bunny", dataNodes.UUID()));
    cntxt->addToSync(cntxt->createNodeFromType("Instances", "Instances", root.UUID()));
  }

  cntxt->syncContext();

  gvt::core::DBNodeH dataNodes = root["Data"];
  gvt::core::DBNodeH instNodes = root["Instances"];

  gvt::core::DBNodeH bunnyMeshNode = dataNodes.getChildren()[0];

  {

    std::string objPath = std::string("../data/geom/bunny.obj");
    if (cmd.isSet("obj")) {
      objPath = cmd.getValue<std::string>("obj")[0];
    }

    // path assumes binary is run as bin/gvtFileApp
    gvt::render::data::domain::reader::ObjReader objReader(objPath);
    // right now mesh must be converted to gvt format
    Mesh *mesh = objReader.getMesh();
    mesh->generateNormals();

    mesh->computeBoundingBox();
    gvt::core::data::primitives::Box3D *meshbbox = mesh->getBoundingBox();

    // add bunny mesh to the database

    bunnyMeshNode["file"] = objPath;
    bunnyMeshNode["bbox"] = (unsigned long long)meshbbox;
    bunnyMeshNode["ptr"] = (unsigned long long)mesh;

    gvt::core::DBNodeH loc = cntxt->createNode("rank", int(comm->id()));
    bunnyMeshNode["Locations"] += loc;

    cntxt->addToSync(bunnyMeshNode);
  }

  cntxt->syncContext();

  // create the instance
  if (comm->id() == 0) {
    gvt::core::DBNodeH instnode =
        cntxt->createNodeFromType("Instance", "inst", instNodes.UUID());
    gvt::core::DBNodeH meshNode = bunnyMeshNode;
    gvt::core::data::primitives::Box3D *mbox =
        (gvt::core::data::primitives::Box3D *)meshNode["bbox"].value().toULongLong();

    instnode["id"] = 0; // unique id per instance
    instnode["meshRef"] = meshNode.UUID();

    // transform bunny
    float scale = 1.0;
    auto m = new glm::mat4(1.f);
    auto minv = new glm::mat4(1.f);
    auto normi = new glm::mat3(1.f);
    //*m = glm::translate(*m, glm::vec3(0, 0, 0));
    //*m *glm::mat4::createTranslation(0.0, 0.0, 0.0);
    //*m = *m * glm::mat4::createScale(scale, scale, scale);
    *m = glm::scale(*m, glm::vec3(scale, scale, scale));

    instnode["mat"] = (unsigned long long)m;
    *minv = glm::inverse(*m);
    instnode["matInv"] = (unsigned long long)minv;
    *normi = glm::transpose(glm::inverse(glm::mat3(*m)));
    instnode["normi"] = (unsigned long long)normi;

    // transform mesh bounding box
    auto il = glm::vec3((*m) * glm::vec4(mbox->bounds_min, 1.f));
    auto ih = glm::vec3((*m) * glm::vec4(mbox->bounds_max, 1.f));
    gvt::core::data::primitives::Box3D *ibox =
        new gvt::core::data::primitives::Box3D(il, ih);
    instnode["bbox"] = (unsigned long long)ibox;
    instnode["centroid"] = ibox->centroid();
    cntxt->addToSync(instnode);
  }

  cntxt->syncContext();

  // add a light
  gvt::core::DBNodeH lightNodes =
      cntxt->createNodeFromType("Lights", "Lights", root.UUID());

  // area Light
  // gvt::core::DBNodeH lightNode = cntxt->createNodeFromType("AreaLight", "light",
  // lightNodes.UUID());
  // lightNode["position"] = glm::vec3(-0.2, 0.1, 0.9, 0.0);
  // lightNode["color"] = glm::vec3(1.0, 1.0, 1.0, 0.0);
  // lightNode["normal"] = glm::vec3(0.0, 0.0, 1.0, 0.0);
  // lightNode["width"] = float(0.05);
  // lightNode["height"] = float(0.05);

  gvt::core::DBNodeH lightNode =
      cntxt->createNodeFromType("PointLight", "light", lightNodes.UUID());
  lightNode["position"] = glm::vec3(0.0, 0.1, 0.5);
  lightNode["color"] = glm::vec3(1.0, 1.0, 1.0);

  // set the camera
  gvt::core::DBNodeH camNode = cntxt->createNodeFromType("Camera", "cam", root.UUID());
  camNode["eyePoint"] = glm::vec3(0.0, 0.1, 0.3);
  camNode["focus"] = glm::vec3(0.0, 0.1, -0.3);
  camNode["upVector"] = glm::vec3(0.0, 1.0, 0.0);
  camNode["fov"] = (float)(45.0 * M_PI / 180.0);
  camNode["rayMaxDepth"] = (int)1;
  camNode["raySamples"] = (int)1;
  camNode["jitterWindowSize"] = (float)0;

  // set image width/height
  gvt::core::DBNodeH filmNode = cntxt->createNodeFromType("Film", "film", root.UUID());
  filmNode["width"] = 512;
  filmNode["height"] = 512;
  filmNode["outputPath"] = (std::string) "bunny";

  if (cmd.isSet("eye")) {
    std::vector<float> eye = cmd.getValue<float>("eye");
    camNode["eyePoint"] = glm::vec3(eye[0], eye[1], eye[2]);
  }

  if (cmd.isSet("look")) {
    std::vector<float> eye = cmd.getValue<float>("look");
    camNode["focus"] = glm::vec3(eye[0], eye[1], eye[2]);
  }
  if (cmd.isSet("wsize")) {
    std::vector<int> wsize = cmd.getValue<int>("wsize");
    filmNode["width"] = wsize[0];
    filmNode["height"] = wsize[1];
  }
  if (cmd.isSet("output")) {
    std::vector<std::string> output = cmd.getValue<std::string>("output");
    filmNode["outputPath"] = output[0];
  }

  gvt::core::DBNodeH schedNode =
      cntxt->createNodeFromType("Schedule", "Enzosched", root.UUID());
  if (cmd.isSet("domain"))
    schedNode["type"] = gvt::render::scheduler::Domain;
  else
    schedNode["type"] = gvt::render::scheduler::Image;

// schedNode["type"] = gvt::render::scheduler::Domain;

#ifdef GVT_RENDER_ADAPTER_EMBREE
  int adapterType = gvt::render::adapter::Embree;
#elif GVT_RENDER_ADAPTER_MANTA
  int adapterType = gvt::render::adapter::Manta;
#elif GVT_RENDER_ADAPTER_OPTIX
  int adapterType = gvt::render::adapter::Optix;
#else
  GVT_DEBUG(DBG_ALWAYS, "ERROR: missing valid adapter");
#endif

  schedNode["adapter"] = adapterType;
   return true;
}


void setCamera() {
  gvt::render::RenderContext *cntxt = gvt::render::RenderContext::instance();
  gvt::core::DBNodeH camNode = cntxt->getRootNode()["Camera"];
  gvt::core::DBNodeH filmNode = cntxt->getRootNode()["Film"];

  std::shared_ptr<gvt::render::data::scene::gvtPerspectiveCamera> mycamera =
      std::make_shared<gvt::render::data::scene::gvtPerspectiveCamera>();
  glm::vec3 cameraposition = camNode["eyePoint"].value().tovec3();
  glm::vec3 focus = camNode["focus"].value().tovec3();
  float fov = camNode["fov"].value().toFloat();
  glm::vec3 up = camNode["upVector"].value().tovec3();
  int rayMaxDepth = camNode["rayMaxDepth"].value().toInteger();
  int raySamples = camNode["raySamples"].value().toInteger();
  //float jitterWindowSize = camNode["jitterWindowSize"].value().toFloat();

  mycamera->lookAt(cameraposition, focus, up);
  mycamera->setMaxDepth(rayMaxDepth);
  mycamera->setSamples(raySamples);
  //mycamera->setJitterWindowSize(jitterWindowSize);
  mycamera->setFOV(fov);
  mycamera->setFilmsize(filmNode["width"].value().toInteger(),
                        filmNode["height"].value().toInteger());

  cntxt->setCamera(mycamera);
}

void setImage() {
  gvt::render::RenderContext *cntxt = gvt::render::RenderContext::instance();
  gvt::core::DBNodeH filmNode = cntxt->getRootNode()["Film"];
  cntxt->setComposite(std::make_shared<gvt::render::composite::IceTComposite>(
      filmNode["width"].value().toInteger(), filmNode["height"].value().toInteger()));
}

int main(int argc, char *argv[]) {
  gvt::comm::scomm::init(argc, argv);
  std::shared_ptr<gvt::comm::communicator> comm = gvt::comm::communicator::singleton();
  gvt::render::RenderContext *cntxt = gvt::render::RenderContext::instance();

  if (!setContext(argc, argv)) return 0;
  setCamera();
  setImage();

  std::shared_ptr<gvt::tracer::Tracer> tracer =
      std::make_shared<gvt::tracer::DomainTracer>();

  cntxt->setTracer(tracer);
  std::shared_ptr<gvt::render::composite::ImageComposite> composite_buffer =
      cntxt->getComposite<gvt::render::composite::ImageComposite>();

  int nFrames =100;
  for (int ii = 0; ii < nFrames; ii++) {

    gvt::core::time::timer t(true, "Frame timer");
    composite_buffer->reset();
    (*tracer)();

//    glm::vec3 light = cntxt->getRootNode()["Lights"].getChildren()[0]["position"].value().tovec3();
//	Rotate(light,cntxt->getCamera()->getFocalPoint(), ((360*2)/(nFrames-1))*M_PI/180, cntxt->getCamera()->getUpVector());
//	cntxt->getRootNode()["Lights"].getChildren()[0]["position"] = light;
//
//	std::shared_ptr<gvt::render::data::scene::gvtCameraBase> camera = cntxt->getCamera();
//	glm::vec3 cameraposition = cntxt->getRootNode()["Camera"]["eyePoint"].value().tovec3();
//	glm::vec3 focus = cntxt->getRootNode()["Camera"]["focus"].value().tovec3();
//	Rotate(cameraposition, focus, ((360*2)/(nFrames-1))*M_PI/180, cntxt->getCamera()->getUpVector());
//	camera->lookAt(cameraposition, focus, cntxt->getCamera()->getUpVector());
//	cntxt->getRootNode()["Camera"]["eyePoint"] = cameraposition;

  // composite_buffer->write(cntxt->getRootNode()["Film"]["outputPath"].value().toString()+std::to_string(ii));



  }

  composite_buffer->write(cntxt->getRootNode()["Film"]["outputPath"].value().toString());
  comm->terminate();

  return 0;
}
