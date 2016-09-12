#include <iostream>

#include <gvt/core/Context.h>
#include <gvt/core/comm/acommunicator.h>
#include <gvt/core/comm/message.h>
#include <gvt/render/Context.h>

#include <gvt/render/composite/IceTComposite.h>
#include <gvt/render/composite/ImageComposite.h>
#include <gvt/render/tracer/Domain/DomainTracer.h>
#include <gvt/render/tracer/Image/ImageTracer.h>

#include "ParseCommandLine.h"

#include <chrono>
#include <memory>
#include <thread>

/* NEED TO REDUCE THIS */

#include <algorithm>
#include <gvt/core/Math.h>
#include <gvt/core/mpi/Wrapper.h>
#include <gvt/render/Context.h>
#include <gvt/render/Schedulers.h>
#include <gvt/render/Types.h>
#include <gvt/render/data/Domains.h>
#include <set>
#include <vector>

#include <tbb/task_scheduler_init.h>
#include <thread>

#ifdef GVT_RENDER_ADAPTER_EMBREE
#include <gvt/render/adapter/embree/Wrapper.h>
#endif

#ifdef GVT_RENDER_ADAPTER_MANTA
#include <gvt/render/adapter/manta/Wrapper.h>
#endif

#ifdef GVT_RENDER_ADAPTER_OPTIX
#include <gvt/render/adapter/optix/Wrapper.h>
#endif

#ifdef GVT_USE_MPE
#include "mpe.h"
#endif
#include <gvt/core/data/primitives/BBox.h>
#include <gvt/render/algorithm/Tracers.h>
#include <gvt/render/data/Primitives.h>
#include <gvt/render/data/scene/Image.h>
#include <gvt/render/data/scene/gvtCamera.h>

#include <boost/range/algorithm.hpp>

#include <iostream>

#ifdef __USE_TAU
#include <TAU.h>
#endif

/* NEED TO REDUCE INCLUDES ABOVE */

using namespace std;
using namespace gvt::render;

using namespace gvt::core::mpi;
using namespace gvt::render::data::scene;
using namespace gvt::render::schedule;
using namespace gvt::render::data::primitives;

void setContext(int argc, char *argv[]) {

  ParseCommandLine cmd("gvtSimple");

  cmd.addoption("wsize", ParseCommandLine::INT, "Window size", 2);
  cmd.addoption("eye", ParseCommandLine::FLOAT, "Camera position", 3);
  cmd.addoption("look", ParseCommandLine::FLOAT, "Camera look at", 3);
  cmd.addoption("lpos", ParseCommandLine::FLOAT, "Light position", 3);
  cmd.addoption("lcolor", ParseCommandLine::FLOAT, "Light color", 3);
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

  // MPI_Init(&argc, &argv);
  // MPI_Pcontrol(0);
  // int rank = -1;
  //   MPI_Comm_rank(MPI_COMM_WORLD, &rank);
  // #ifdef GVT_USE_MPE
  //   // MPE_Init_log();
  //   int readstart, readend;
  //   int renderstart, renderend;
  //   MPE_Log_get_state_eventIDs(&readstart, &readend);
  //   MPE_Log_get_state_eventIDs(&renderstart, &renderend);
  //   if (rank == 0) {
  //     MPE_Describe_state(readstart, readend, "Initialize context state", "red");
  //     MPE_Describe_state(renderstart, renderend, "Render", "yellow");
  //   }
  //   MPI_Pcontrol(1);
  //   MPE_Log_event(readstart, 0, NULL);
  // #endif

  std::shared_ptr<gvt::comm::acommunicator> comm =
      gvt::comm::acommunicator::instance(argc, argv);
  gvt::render::RenderContext *cntxt = gvt::render::RenderContext::instance();
  if (cntxt == NULL) {
    std::cout << "Something went wrong initializing the context" << std::endl;
    exit(0);
  }

  gvt::core::DBNodeH root = cntxt->getRootNode();

  // mix of cones and cubes

  // if (rank == 0) {
  gvt::core::DBNodeH dataNodes =
      cntxt->addToSync(cntxt->createNodeFromType("Data", "Data", root.UUID()));
  cntxt->addToSync(cntxt->createNodeFromType("Mesh", "conemesh", dataNodes.UUID()));
  cntxt->addToSync(cntxt->createNodeFromType("Mesh", "cubemesh", dataNodes.UUID()));
  gvt::core::DBNodeH instNodes =
      cntxt->addToSync(cntxt->createNodeFromType("Instances", "Instances", root.UUID()));
  // }

  // cntxt->syncContext();

  // gvt::core::DBNodeH dataNodes = root["Data"];
  // gvt::core::DBNodeH instNodes = root["Instances"];

  gvt::core::DBNodeH coneMeshNode = dataNodes.getChildren()[0];
  gvt::core::DBNodeH cubeMeshNode = dataNodes.getChildren()[1];

  {
    Material *m = new Material();
    m->type = LAMBERT;
    // m->type = EMBREE_MATERIAL_MATTE;
    m->kd = glm::vec3(1.0, 1.0, 1.0);
    m->ks = glm::vec3(1.0, 1.0, 1.0);
    m->alpha = 0.5;

    // m->type = EMBREE_MATERIAL_METAL;
    // copper metal
    m->eta = glm::vec3(.19, 1.45, 1.50);
    m->k = glm::vec3(3.06, 2.40, 1.88);
    m->roughness = 0.05;

    Mesh *mesh = new Mesh(m);
    int numPoints = 7;
    glm::vec3 points[6];
    points[0] = glm::vec3(0.5, 0.0, 0.0);
    points[1] = glm::vec3(-0.5, 0.5, 0.0);
    points[2] = glm::vec3(-0.5, 0.25, 0.433013);
    points[3] = glm::vec3(-0.5, -0.25, 0.43013);
    points[4] = glm::vec3(-0.5, -0.5, 0.0);
    points[5] = glm::vec3(-0.5, -0.25, -0.433013);
    points[6] = glm::vec3(-0.5, 0.25, -0.433013);

    for (int i = 0; i < numPoints; i++) {
      mesh->addVertex(points[i]);
    }
    mesh->addFace(1, 2, 3);
    mesh->addFace(1, 3, 4);
    mesh->addFace(1, 4, 5);
    mesh->addFace(1, 5, 6);
    mesh->addFace(1, 6, 7);
    mesh->addFace(1, 7, 2);
    mesh->generateNormals();

    // calculate bbox
    glm::vec3 lower = points[0], upper = points[0];
    for (int i = 1; i < numPoints; i++) {
      for (int j = 0; j < 3; j++) {
        lower[j] = (lower[j] < points[i][j]) ? lower[j] : points[i][j];
        upper[j] = (upper[j] > points[i][j]) ? upper[j] : points[i][j];
      }
    }
    gvt::core::data::primitives::Box3D *meshbbox =
        new gvt::core::data::primitives::Box3D(lower, upper);

    // add cone mesh to the database
    gvt::core::Variant meshvariant(mesh);
    //    std::cout << "meshvariant " << meshvariant << std::endl;
    coneMeshNode["file"] = string("/fake/path/to/cone");
    coneMeshNode["bbox"] = (unsigned long long)meshbbox;
    coneMeshNode["ptr"] = (unsigned long long)mesh;

    gvt::core::DBNodeH loc =
        cntxt->createNode("rank", (comm->id() & 1 == 0) ? (int)comm->id() : 0);
    coneMeshNode["Locations"] += loc;

    // cntxt->addToSync(coneMeshNode);
  }

  {

    Material *m = new Material();
    m->type = LAMBERT;
    // m->type = EMBREE_MATERIAL_MATTE;
    m->kd = glm::vec3(1.0, 1.0, 1.0);
    m->ks = glm::vec3(1.0, 1.0, 1.0);
    m->alpha = 0.5;

    // m->type = EMBREE_MATERIAL_METAL;
    // copper metal
    m->eta = glm::vec3(.19, 1.45, 1.50);
    m->k = glm::vec3(3.06, 2.40, 1.88);
    m->roughness = 0.05;

    Mesh *mesh = new Mesh(m);

    int numPoints = 24;
    glm::vec3 points[24];
    points[0] = glm::vec3(-0.5, -0.5, 0.5);
    points[1] = glm::vec3(0.5, -0.5, 0.5);
    points[2] = glm::vec3(0.5, 0.5, 0.5);
    points[3] = glm::vec3(-0.5, 0.5, 0.5);
    points[4] = glm::vec3(-0.5, -0.5, -0.5);
    points[5] = glm::vec3(0.5, -0.5, -0.5);
    points[6] = glm::vec3(0.5, 0.5, -0.5);
    points[7] = glm::vec3(-0.5, 0.5, -0.5);

    points[8] = glm::vec3(0.5, 0.5, 0.5);
    points[9] = glm::vec3(-0.5, 0.5, 0.5);
    points[10] = glm::vec3(0.5, 0.5, -0.5);
    points[11] = glm::vec3(-0.5, 0.5, -0.5);

    points[12] = glm::vec3(-0.5, -0.5, 0.5);
    points[13] = glm::vec3(0.5, -0.5, 0.5);
    points[14] = glm::vec3(-0.5, -0.5, -0.5);
    points[15] = glm::vec3(0.5, -0.5, -0.5);

    points[16] = glm::vec3(0.5, -0.5, 0.5);
    points[17] = glm::vec3(0.5, 0.5, 0.5);
    points[18] = glm::vec3(0.5, -0.5, -0.5);
    points[19] = glm::vec3(0.5, 0.5, -0.5);

    points[20] = glm::vec3(-0.5, -0.5, 0.5);
    points[21] = glm::vec3(-0.5, 0.5, 0.5);
    points[22] = glm::vec3(-0.5, -0.5, -0.5);
    points[23] = glm::vec3(-0.5, 0.5, -0.5);

    for (int i = 0; i < numPoints; i++) {
      mesh->addVertex(points[i]);
    }
    // faces are 1 indexed
    mesh->addFace(1, 2, 3);
    mesh->addFace(1, 3, 4);

    mesh->addFace(17, 19, 20);
    mesh->addFace(17, 20, 18);

    mesh->addFace(6, 5, 8);
    mesh->addFace(6, 8, 7);

    mesh->addFace(23, 21, 22);
    mesh->addFace(23, 22, 24);

    mesh->addFace(10, 9, 11);
    mesh->addFace(10, 11, 12);

    mesh->addFace(13, 15, 16);
    mesh->addFace(13, 16, 14);

    mesh->generateNormals();

    // calculate bbox
    glm::vec3 lower = points[0], upper = points[0];
    for (int i = 1; i < numPoints; i++) {
      for (int j = 0; j < 3; j++) {
        lower[j] = (lower[j] < points[i][j]) ? lower[j] : points[i][j];
        upper[j] = (upper[j] > points[i][j]) ? upper[j] : points[i][j];
      }
    }
    gvt::core::data::primitives::Box3D *meshbbox =
        new gvt::core::data::primitives::Box3D(lower, upper);

    // add cube mesh to the database
    cubeMeshNode["file"] = string("/fake/path/to/cube");
    cubeMeshNode["bbox"] = (unsigned long long)meshbbox;
    cubeMeshNode["ptr"] = (unsigned long long)mesh;

    gvt::core::DBNodeH loc =
        cntxt->createNode("rank", (comm->id() & 1 == 1) ? (int)comm->id() : 1);
    cubeMeshNode["Locations"] += loc;

    // cntxt->addToSync(cubeMeshNode);
  }

  // cntxt->syncContext();

  // if (rank == 0) {
  // create a NxM grid of alternating cones / cubes, offset using i and j
  int instId = 0;
  int ii[2] = { -2, 3 }; // i range
  int jj[2] = { -2, 3 }; // j range
  for (int i = ii[0]; i < ii[1]; i++) {
    for (int j = jj[0]; j < jj[1]; j++) {
      gvt::core::DBNodeH instnode =
          cntxt->createNodeFromType("Instance", "inst", instNodes.UUID());
      // gvt::core::DBNodeH meshNode = (instId % 2) ? coneMeshNode : cubeMeshNode;
      gvt::core::DBNodeH meshNode = (instId % 2) ? cubeMeshNode : coneMeshNode;
      gvt::core::data::primitives::Box3D *mbox =
          (gvt::core::data::primitives::Box3D *)meshNode["bbox"].value().toULongLong();

      instnode["id"] = instId++;
      instnode["meshRef"] = meshNode.UUID();

      auto m = new glm::mat4(1.f);
      auto minv = new glm::mat4(1.f);
      auto normi = new glm::mat3(1.f);
      //*m *glm::mat4::createTranslation(0.0, i * 0.5, j * 0.5);
      *m = glm::translate(*m, glm::vec3(0.0, i * 0.5, j * 0.5));
      //*m = *m * glm::mat4::createScale(0.4, 0.4, 0.4);
      *m = glm::scale(*m, glm::vec3(0.4, 0.4, 0.4));

      instnode["mat"] = (unsigned long long)m;
      *minv = glm::inverse(*m);
      instnode["matInv"] = (unsigned long long)minv;
      *normi = glm::transpose(glm::inverse(glm::mat3(*m)));
      instnode["normi"] = (unsigned long long)normi;
      auto il = glm::vec3((*m) * glm::vec4(mbox->bounds_min, 1.f));
      auto ih = glm::vec3((*m) * glm::vec4(mbox->bounds_max, 1.f));
      gvt::core::data::primitives::Box3D *ibox =
          new gvt::core::data::primitives::Box3D(il, ih);
      instnode["bbox"] = (unsigned long long)ibox;
      instnode["centroid"] = ibox->centroid();

      // cntxt->addToSync(instnode);
    }
  }
  // }

  // cntxt->syncContext();

  // add lights, camera, and film to the database
  gvt::core::DBNodeH lightNodes =
      cntxt->createNodeFromType("Lights", "Lights", root.UUID());
#if 1
  gvt::core::DBNodeH lightNode =
      cntxt->createNodeFromType("PointLight", "conelight", lightNodes.UUID());
  lightNode["position"] = glm::vec3(1.0, 0.0, -1.0);
  lightNode["color"] = glm::vec3(1.0, 1.0, 1.0);
#else
  gvt::core::DBNodeH lightNode =
      cntxt->createNodeFromType("AreaLight", "AreaLight", lightNodes.UUID());

  lightNode["position"] = glm::vec3(1.0, 0.0, 0.0);
  lightNode["normal"] = glm::vec3(-1.0, 0.0, 0.0);
  lightNode["width"] = 2.f;
  lightNode["height"] = 2.f;
  lightNode["color"] = glm::vec3(1.0, 1.0, 1.0);
#endif
  // second light just for fun
  // gvt::core::DBNodeH lN2 = cntxt->createNodeFromType("PointLight", "conelight",
  // lightNodes.UUID());
  // lN2["position"] = glm::vec3(2.0, 2.0, 2.0, 0.0);
  // lN2["color"] = glm::vec3(0.0, 0.0, 0.0, 0.0);

  gvt::core::DBNodeH camNode =
      cntxt->createNodeFromType("Camera", "conecam", root.UUID());
  camNode["eyePoint"] = glm::vec3(4.0, 0.0, 0.0);
  camNode["focus"] = glm::vec3(0.0, 0.0, 0.0);
  camNode["upVector"] = glm::vec3(0.0, 1.0, 0.0);
  camNode["fov"] = (float)(45.0 * M_PI / 180.0);
  camNode["rayMaxDepth"] = (int)1;
  camNode["raySamples"] = (int)1;
  camNode["jitterWindowSize"] = (float)0.5;

  gvt::core::DBNodeH filmNode =
      cntxt->createNodeFromType("Film", "conefilm", root.UUID());
  filmNode["width"] = 512;
  filmNode["height"] = 512;
  filmNode["outputPath"] = (std::string) "simple";

  if (cmd.isSet("lpos")) {
    std::vector<float> pos = cmd.getValue<float>("lpos");
    lightNode["position"] = glm::vec3(pos[0], pos[1], pos[2]);
  }
  if (cmd.isSet("lcolor")) {
    std::vector<float> color = cmd.getValue<float>("lcolor");
    lightNode["color"] = glm::vec3(color[0], color[1], color[2]);
  }

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
  float jitterWindowSize = camNode["jitterWindowSize"].value().toFloat();

  mycamera->lookAt(cameraposition, focus, up);
  mycamera->setMaxDepth(rayMaxDepth);
  mycamera->setSamples(raySamples);
  mycamera->setJitterWindowSize(jitterWindowSize);
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

  std::shared_ptr<gvt::comm::acommunicator> comm =
      gvt::comm::acommunicator::instance(argc, argv);

  gvt::render::RenderContext *cntxt = gvt::render::RenderContext::instance();

  setContext(argc, argv);
  setCamera();
  setImage();

#if 0
  if (comm->id() == 0) {
    std::shared_ptr<gvt::comm::Message> msg =
        std::make_shared<gvt::comm::Message>(sizeof(int));
    int data = 0;
    while (data < 9) {
      msg->setcontent(&data, sizeof(int));
      comm->send(msg, comm->maxid() - 1);
      while (!comm->hasMessages())
        ;
      std::shared_ptr<gvt::comm::Message> msg2 = comm->popMessage();
      data = *(int *)(msg2->msg_ptr()) + 1;
      // std::cout << "Data : " << data << std::endl;
    }
    std::cout << "Out of the loop " << comm->id() << std::endl;
  }

  if (comm->maxid() > 1 && comm->id() == comm->maxid() - 1) {
    std::shared_ptr<gvt::comm::Message> msg =
        std::make_shared<gvt::comm::Message>(sizeof(int));
    int data = 0;
    while (data < 9) {

      while (!comm->hasMessages())
        ;
      std::shared_ptr<gvt::comm::Message> msg2 = comm->popMessage();
      data = *(int *)(msg2->msg_ptr()) + 1;
      msg->setcontent(&data, sizeof(int));
      comm->send(msg, 0);
    }
    std::cout << "Out of the loop " << comm->id() << std::endl;
  }
#endif
  std::shared_ptr<gvt::tracer::DomainTracer> tracer =
      std::make_shared<gvt::tracer::DomainTracer>();
  std::shared_ptr<gvt::render::composite::ImageComposite> composite_buffer =
      cntxt->getComposite<gvt::render::composite::ImageComposite>();
  for (int ii = 0; ii < 10; ii++) {
    composite_buffer->reset();
    (*tracer)();
  }

  composite_buffer->write(cntxt->getRootNode()["Film"]["outputPath"].value().toString());
  comm->terminate();
  return 0;
}
