#include "apps/render/TestScenes.h"
#include "gvt/core/Math.h"
#include "gvt/core/Uuid.h"
#include "gvt/render/RenderContext.h"
#include "gvt/render/data/reader/ObjReader.h"
#include "gvt/render/data/primitives/Mesh.h"
#include "gvt/render/unit/MpiRenderer.h"

#include <ply.h>

using namespace apps::render;
using namespace gvt::core;
using namespace gvt::render;
using namespace gvt::render::data::primitives;
using namespace gvt::render::data::scene;

#ifndef MIN
#define MIN(a, b) ((a < b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a, b) ((a > b) ? (a) : (b))
#endif

typedef struct Vertex {
  float x, y, z;
  float nx, ny, nz;
  void *other_props; /* other properties */
} Vertex;

typedef struct Face {
  unsigned char nverts; /* number of vertex indices in list */
  int *verts;           /* vertex index list */
  void *other_props;    /* other properties */
} Face;

PlyProperty vert_props[] = {
  /* list of property information for a vertex */
  { "x", Float32, Float32, offsetof(Vertex, x), 0, 0, 0, 0 },
  { "y", Float32, Float32, offsetof(Vertex, y), 0, 0, 0, 0 },
  { "z", Float32, Float32, offsetof(Vertex, z), 0, 0, 0, 0 },
  { "nx", Float32, Float32, offsetof(Vertex, nx), 0, 0, 0, 0 },
  { "ny", Float32, Float32, offsetof(Vertex, ny), 0, 0, 0, 0 },
  { "nz", Float32, Float32, offsetof(Vertex, nz), 0, 0, 0, 0 },
};

PlyProperty face_props[] = {
  /* list of property information for a face */
  { "vertex_indices", Int32, Int32, offsetof(Face, verts), 1, Uint8, Uint8, offsetof(Face, nverts) },
};

static Vertex **vlist;
static Face **flist;

TestScenes::TestScenes(const gvt::render::unit::MpiRendererOptions &options) : options(options) {
  renderContext = gvt::render::RenderContext::instance();
}

void TestScenes::makePlyDatabase() {
  // mess I use to open and read the ply file with the c utils I found.
  PlyFile *in_ply;
  Vertex *vert;
  Face *face;
  int elem_count, nfaces, nverts;
  int i, j, k;
  float xmin, ymin, zmin, xmax, ymax, zmax;
  char *elem_name;
  FILE *myfile;
  char txt[16];
  std::string temp;
  std::string filename, filepath, rootdir;
  // rootdir = "/work/01197/semeraro/maverick/DAVEDATA/EnzoPlyData/";
  rootdir = options.infile;

  gvt::core::DBNodeH root = renderContext->getRootNode();
  gvt::core::DBNodeH dataNodes = renderContext->createNodeFromType("Data", "Data", root.UUID());
  gvt::core::DBNodeH instNodes = renderContext->createNodeFromType("Instances", "Instances", root.UUID());

  // Enzo isosurface...
  const int numPlyFiles = 8;
  for (k = 0; k < numPlyFiles; k++) {
    sprintf(txt, "%d", k);
    filename = "block";
    filename += txt;
    gvt::core::DBNodeH plyMeshNode = renderContext->createNodeFromType("Mesh", filename.c_str(), dataNodes.UUID());
    // read in some ply data and get ready to load it into the mesh
    // filepath = rootdir + "block" + std::string(txt) + ".ply";
    filepath = rootdir + filename + ".ply";
    myfile = fopen(filepath.c_str(), "r");
    if (!myfile) {
      printf("%s not found\n", filepath.c_str());
      exit(1);
    }
    in_ply = read_ply(myfile);
    for (i = 0; i < in_ply->num_elem_types; i++) {
      elem_name = setup_element_read_ply(in_ply, i, &elem_count);
      temp = elem_name;
      if (temp == "vertex") {
        vlist = (Vertex **)malloc(sizeof(Vertex *) * elem_count);
        nverts = elem_count;
        setup_property_ply(in_ply, &vert_props[0]);
        setup_property_ply(in_ply, &vert_props[1]);
        setup_property_ply(in_ply, &vert_props[2]);
        for (j = 0; j < elem_count; j++) {
          vlist[j] = (Vertex *)malloc(sizeof(Vertex));
          get_element_ply(in_ply, (void *)vlist[j]);
        }
      } else if (temp == "face") {
        flist = (Face **)malloc(sizeof(Face *) * elem_count);
        nfaces = elem_count;
        setup_property_ply(in_ply, &face_props[0]);
        for (j = 0; j < elem_count; j++) {
          flist[j] = (Face *)malloc(sizeof(Face));
          get_element_ply(in_ply, (void *)flist[j]);
        }
      }
    }
    close_ply(in_ply);
    // smoosh data into the mesh object
    {
      Mesh *mesh = new Mesh(new Lambert(glm::vec3(1.0, 1.0, 1.0)));
      vert = vlist[0];
      xmin = vert->x;
      ymin = vert->y;
      zmin = vert->z;
      xmax = vert->x;
      ymax = vert->y;
      zmax = vert->z;

      for (i = 0; i < nverts; i++) {
        vert = vlist[i];
        xmin = MIN(vert->x, xmin);
        ymin = MIN(vert->y, ymin);
        zmin = MIN(vert->z, zmin);
        xmax = MAX(vert->x, xmax);
        ymax = MAX(vert->y, ymax);
        zmax = MAX(vert->z, zmax);
        mesh->addVertex(glm::vec3(vert->x, vert->y, vert->z));
      }
      glm::vec3 lower(xmin, ymin, zmin);
      glm::vec3 upper(xmax, ymax, zmax);
      Box3D *meshbbox = new gvt::render::data::primitives::Box3D(lower, upper);
      // add faces to mesh
      for (i = 0; i < nfaces; i++) {
        face = flist[i];
        mesh->addFace(face->verts[0] + 1, face->verts[1] + 1, face->verts[2] + 1);
      }
      mesh->generateNormals();
      // add Enzo mesh to the database
      // plyMeshNode["file"] = string("/work/01197/semeraro/maverick/DAVEDATA/EnzoPlyDATA/Block0.ply");
      plyMeshNode["file"] = std::string(filepath);
      plyMeshNode["bbox"] = (unsigned long long)meshbbox;
      plyMeshNode["ptr"] = (unsigned long long)mesh;
    }
    // add instance
    gvt::core::DBNodeH instnode = renderContext->createNodeFromType("Instance", "inst", instNodes.UUID());
    gvt::core::DBNodeH meshNode = plyMeshNode;
    Box3D *mbox = (Box3D *)meshNode["bbox"].value().toULongLong();
    instnode["id"] = k;
    instnode["meshRef"] = meshNode.UUID();
    auto m = new glm::mat4(1.f);
    auto minv = new glm::mat4(1.f);
    auto normi = new glm::mat3(1.f);
    instnode["mat"] = (unsigned long long)m;
    *minv = glm::inverse(*m);
    instnode["matInv"] = (unsigned long long)minv;
    *normi = glm::transpose(glm::inverse(glm::mat3(*m)));
    instnode["normi"] = (unsigned long long)normi;
    auto il = glm::vec3((*m) * glm::vec4(mbox->bounds_min, 1.f));
    auto ih = glm::vec3((*m) * glm::vec4(mbox->bounds_max, 1.f));
    Box3D *ibox = new gvt::render::data::primitives::Box3D(il, ih);
    instnode["bbox"] = (unsigned long long)ibox;
    instnode["centroid"] = ibox->centroid();
  }

  // add lights, camera, and film to the database
  gvt::core::DBNodeH lightNodes = renderContext->createNodeFromType("Lights", "Lights", root.UUID());
  gvt::core::DBNodeH lightNode = renderContext->createNodeFromType("PointLight", "conelight", lightNodes.UUID());
  lightNode["position"] = glm::vec3(512.0, 512.0, 2048.0);
  lightNode["color"] = glm::vec3(100.0, 100.0, 500.0);

  // camera
  // Point4f eye(512.0, 512.0, 4096.0, 1.0);
  // Point4f focus(512.0, 512.0, 0.0, 1.0);
  // Vector4f upVector(0.0, 1.0, 0.0, 0.0);
  // float fov = (float)(25.0 * M_PI / 180.0);
  // Uuid cameraNodeId = createCameraNode(eye, focus, upVector, fov, options.width, options.height);

  // film
  Uuid filmNodeId = createFilmNode(options.width, options.height, "");

  // scheduler
  Uuid scheduleNodeId = createScheduleNode(options.schedulerType, options.adapterType);
}

void TestScenes::makeObjDatabase() {
  // create data node
  Uuid dataNodeId = createNode("Data", "Data");
  std::string objName("obj_name");

  // create instances node
  Uuid instancesNodeId = createNode("Instances", "Instances");

  // add instances
  Box3D meshBounds = getMeshBounds(options.infile);
  glm::vec3 extent = meshBounds.extent();

  const float gapX = extent[0] * 0.2f;
  const float gapY = extent[1] * 0.2f;
  const float gapZ = extent[2] * 0.2f;

  int instanceCountX = options.instanceCountX;
  int instanceCountY = options.instanceCountY;
  int instanceCountZ = options.instanceCountZ;

  glm::vec3 minPos((extent[0] + gapX) * instanceCountX * -0.5f, (extent[1] + gapY) * instanceCountY * -0.5f,
                   (extent[2] + gapZ) * instanceCountZ * -0.5f);

  int instanceId = 0;
  for (int z = 0; z < instanceCountZ; ++z) {
    for (int y = 0; y < instanceCountY; ++y) {
      for (int x = 0; x < instanceCountX; ++x) {
        // add a mesh
        Uuid meshNodeId = addMesh(dataNodeId, "mesh_data", options.infile);
        // int instanceId = y * 2 + x;
        glm::mat4 *m = new glm::mat4(1.f);
        *m = glm::translate(*m, glm::vec3(minPos[0] + x * (extent[0] + gapX), minPos[1] + y * (extent[1] + gapY),
                                          minPos[2] + z * (extent[2] + gapZ)));
        Uuid instanceUuid = addInstance(instancesNodeId, meshNodeId, instanceId++, objName, m);
      }
    }
  }

  // create lights node
  Uuid lightsNodeId = createNode("Lights", "Lights");

  // add lights
  glm::vec3 lightPosition(0.0, 0.1, 0.5);
  glm::vec3 lightColor(1.0, 1.0, 1.0);
  Uuid lightNodeId = addPointLight(lightsNodeId, "point_light", lightPosition, lightColor);

  // // create camera node
  // Point4f eye(0.0, 0.5, 1.2, 1.0);
  // Point4f focus(0.0, 0.0, 0.0, 1.0);
  // Vector4f upVector(0.0, 1.0, 0.0, 0.0);
  // float fov = (45.0 * M_PI / 180.0);
  // Uuid cameraNodeId = createCameraNode(eye, focus, upVector, fov, options.width, options.height);

  // create film node
  Uuid filmNodeId = createFilmNode(options.width, options.height, objName);

  // create the scheduler node
  Uuid scheduleNodeId = createScheduleNode(options.schedulerType, options.adapterType);
}

bool TestScenes::isNodeTypeReserved(const std::string &type) {
  return ((type == std::string("Camera")) || (type == std::string("Film")) || (type == std::string("View")) ||
          (type == std::string("Dataset")) || (type == std::string("Attribute")) || (type == std::string("Mesh")) ||
          (type == std::string("Instance")) || (type == std::string("PointLight")) ||
          (type == std::string("Schedule")));
}

DBNodeH TestScenes::getNode(const Uuid &id) { return renderContext->getNode(id); }

Uuid TestScenes::createNode(const std::string &type, const std::string &name) {
  if (isNodeTypeReserved(type)) {
    std::cout << "error: reserved node type " << type << std::endl;
    exit(1);
  }
  gvt::core::DBNodeH root = renderContext->getRootNode();
  gvt::core::DBNodeH node = renderContext->createNodeFromType(type, name, root.UUID());
  return node.UUID();
}

Uuid TestScenes::addMesh(const Uuid &parentNodeId, const std::string &meshName, const std::string &objFilename) {
  gvt::core::DBNodeH node = renderContext->createNodeFromType("Mesh", meshName, parentNodeId);
  gvt::render::data::domain::reader::ObjReader objReader(objFilename);
  Mesh *mesh = objReader.getMesh();
  mesh->generateNormals();
  mesh->computeBoundingBox();
  Box3D *meshbbox = mesh->getBoundingBox();
  // add mesh to the database
  node["file"] = objFilename;
  node["bbox"] = (unsigned long long)meshbbox;
  node["ptr"] = (unsigned long long)mesh;
  return node.UUID();
}

Box3D TestScenes::getMeshBounds(const std::string &objFilename) {
  gvt::render::data::domain::reader::ObjReader objReader(objFilename);
  Mesh *mesh = objReader.getMesh();
  // mesh->generateNormals();
  mesh->computeBoundingBox();
  Box3D *bounds = mesh->getBoundingBox();
  return *bounds;
}

Box3D TestScenes::getMeshBounds(const Uuid &id) {
  gvt::core::DBNodeH meshNode = renderContext->getNode(id);
  // Box3D* bounds = *gvt::core::variant_toBox3DPtr(meshNode["bbox"].value());
  Box3D *bounds = (Box3D *)meshNode["bbox"].value().toULongLong();
  return *bounds;
}

Uuid TestScenes::addInstance(const Uuid &parentNodeId, const Uuid &meshId, int instanceId,
                             const std::string &instanceName, glm::mat4 *transform) {
  gvt::core::DBNodeH node = renderContext->createNodeFromType("Instance", instanceName, parentNodeId);

  gvt::core::DBNodeH meshNode = renderContext->getNode(meshId);
  // Box3D* mbox = *gvt::core::variant_toBox3DPtr(meshNode["bbox"].value());
  Box3D *mbox = (Box3D *)meshNode["bbox"].value().toULongLong();

  node["id"] = instanceId; // unique id per instance
  node["meshRef"] = meshNode.UUID();

  // transform the instance
  auto m = transform;
  auto minv = new glm::mat4(1.f);
  auto normi = new glm::mat3(1.f);

  node["mat"] = (unsigned long long)m;
  *minv = glm::inverse(*m);
  node["matInv"] = (unsigned long long)minv;
  *normi = glm::transpose(glm::inverse(glm::mat3(*m)));
  node["normi"] = (unsigned long long)normi;

  // transform mesh bounding box
  auto il = glm::vec3((*m) * glm::vec4(mbox->bounds_min, 1.f));
  auto ih = glm::vec3((*m) * glm::vec4(mbox->bounds_max, 1.f));
  Box3D *ibox = new gvt::render::data::primitives::Box3D(il, ih);
  node["bbox"] = (unsigned long long)ibox;
  node["centroid"] = ibox->centroid();

  printf("[new instance %d] bounds: min(%.3f %.3f %.3f), max(%.3f %.3f %.3f)\n", instanceId, il[0], il[1], il[2], ih[0],
         ih[1], ih[2]);

  return node.UUID();
}

Uuid TestScenes::addPointLight(const Uuid &parentNodeId, const std::string &lightName, const glm::vec3 &position,
                               const glm::vec3 &color) {
  gvt::core::DBNodeH node = renderContext->createNodeFromType("PointLight", lightName, parentNodeId);
  node["position"] = position;
  node["color"] = color;
  return node.UUID();
}

// Uuid TestScenes::createCameraNode(const Point4f &eye, const Point4f &focus, const Vector4f &upVector, float fov,
//                                    unsigned int width, unsigned int height) {
//
//   gvt::core::DBNodeH root = renderContext->getRootNode();
//
//   gvt::core::DBNodeH node = renderContext->createNodeFromType("Camera", "cam", root.UUID());
//
//   node["eyePoint"] = eye;
//   node["focus"] = focus;
//   node["upVector"] = upVector;
//   node["fov"] = fov;
//
//   camera = new gvtPerspectiveCamera();
//   camera->lookAt(eye, focus, upVector);
//   camera->setFOV(fov);
//   camera->setFilmsize(width, height);
//
//   return node.UUID();
// }

Uuid TestScenes::createFilmNode(int width, int height, const std::string &sceneName) {

  gvt::core::DBNodeH root = renderContext->getRootNode();

  gvt::core::DBNodeH node = renderContext->createNodeFromType("Film", "film", root.UUID());

  node["width"] = width;
  node["height"] = height;

  // image = new Image(width, height, sceneName);

  return node.UUID();
}

Uuid TestScenes::createScheduleNode(int schedulerType, int adapterType) {

  gvt::core::DBNodeH root = renderContext->getRootNode();

  gvt::core::DBNodeH node = renderContext->createNodeFromType("Schedule", "sched", root.UUID());

  node["type"] = schedulerType;
  node["adapter"] = adapterType;

  return node.UUID();
}

