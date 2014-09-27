#include <GVT/utils/wavefront_obj_loader.h>

#include <cstdio>
#include <cstdlib>

#include <GVT/Data/primitives.h>
#include <GVT/Math/GVTMath.h>
#include <objreader/objreader.h>
#include <objreader/usercallbacks.h>

using GVT::Math::Vector4f;
using GVT::Data::Mesh;

namespace GVT {

namespace utils {

// Object Parse Callbacks
//
// These are provided as a bridge between C and C++.
// User data should be of type WavefrontObjLoader*.  The function will
// cast user_data to said pointer type and call the corresponding method.
int AddVertexCallback(float x, float y, float z, float weight, void* user_data);
int AddNormalCallback(float x, float y, float z, void* user_data);
int StartFaceCallback(void* user_data);
int AddToFaceCallback(size_t v, size_t vt, size_t vn, void* user_data);

WavefrontObjLoader::WavefrontObjLoader() { Init(); }

void WavefrontObjLoader::Init() {
  // Setup object parse callbacks.
  memset(&object_parse_callbacks_, sizeof(ObjParseCallbacks), 0);
  object_parse_callbacks_.onVertex = AddVertexCallback;
  object_parse_callbacks_.onNormal = AddNormalCallback;
  object_parse_callbacks_.onStartFace = StartFaceCallback;
  object_parse_callbacks_.onAddToFace = AddToFaceCallback;
  object_parse_callbacks_.userData = reinterpret_cast<void*>(this);
}

// Class methods used by WavefrontObjLoader here.
// We provide a very simple implementation to load faces, vertices,
// and vertex normals.
int WavefrontObjLoader::Load() {
  FILE* file_stream = fopen(file_path_.c_str(), "r");
  int result = ReadObjFile(
      file_stream, const_cast<ObjParseCallbacks*>(&object_parse_callbacks_));
  fclose(file_stream);
  return result;
}

int WavefrontObjLoader::AddVertex(float x, float y, float z, float weight) {
  mesh_->vertices.push_back(Vector4f(x, y, z, weight));
  return 0;
}

int WavefrontObjLoader::AddNormal(float x, float y, float z) {
  mesh_->normals.push_back(Vector4f(x, y, z, 0.0f));
}

int WavefrontObjLoader::StartFace() {
  Mesh::face f = Mesh::face(-1, -1, -1);
  mesh_->faces.push_back(f);
  current_face_vertex_ = 0;
  return 0;
}

int WavefrontObjLoader::AddToFace(size_t v, size_t /* vt */, size_t vn) {
  // Check for out of bounds, remember these are 1 based.
  if (v > mesh_->vertices.size() || vn > mesh_->normals.size()) return -1;
  Mesh::face& f = mesh_->faces.back();
  Mesh::face_to_normals& fn = mesh_->faces_to_normals.back();
  if (current_face_vertex_ < 3) {
    // boost::tuple only has a static accessor, get<N>().
    // This means N has to be known at compile time.
    // Instead of adding needless if/then or switch statements, we get
    // a pointer to the tuple and cast it to int* and access it like an array.
    int* face_tuple = reinterpret_cast<int*>(&f);
    face_tuple[current_face_vertex_] = v - 1;
    int* normal_tuple = reinterpret_cast<int*>(&fn);
    normal_tuple[current_face_vertex_] = vn - 1;
  } else {
    // If #vertices for this face > 3, then process it like a triangle fan.
    // See, TRIANGLE_FAN from fixed function OpenGL.
    Mesh::face g;
    g.get<0>() = f.get<0>();
    g.get<1>() = f.get<2>();
    g.get<2>() = v - 1;
    mesh_->faces.push_back(g);
    Mesh::face_to_normals gn;
    gn.get<0>() = fn.get<0>();
    gn.get<1>() = fn.get<2>();
    gn.get<2>() = vn - 1;
    mesh_->faces_to_normals.push_back(gn);
  }
  ++current_face_vertex_;
  return 0;
}

int AddVertexCallback(float x, float y, float z, float weight,
                      void* user_data) {
  WavefrontObjLoader* loader = reinterpret_cast<WavefrontObjLoader*>(user_data);
  int result = loader->AddVertex(x, y, z, weight);
  return result;
}

int AddNormalCallback(float x, float y, float z, void* user_data) {
  WavefrontObjLoader* loader = reinterpret_cast<WavefrontObjLoader*>(user_data);
  int result = loader->AddNormal(x, y, z);
  return result;
}

int StartFaceCallback(void* user_data) {
  WavefrontObjLoader* loader = reinterpret_cast<WavefrontObjLoader*>(user_data);
  int result = loader->StartFace();
  return result;
}

int AddToFaceCallback(size_t v, size_t vt, size_t vn, void* user_data) {
  WavefrontObjLoader* loader = reinterpret_cast<WavefrontObjLoader*>(user_data);
  int result = loader->AddToFace(v, vt, vn);
  return result;
}

}  // namespace utils

}  // namespace GVT
