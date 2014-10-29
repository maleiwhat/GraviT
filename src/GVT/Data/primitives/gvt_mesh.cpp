/*
 * File:   gvt_mesh.cpp
 * Author: jbarbosa
 *
 * Created on April 20, 2014, 11:01 PM
 */
#include <GVT/Data/primitives/gvt_mesh.h>

#include <GVT/Data/primitives.h>
#include <GVT/Data/primitives/gvt_material_list.h>

using GVT::Data::Color;
using GVT::Data::LightSource;
using GVT::Data::Material;
using GVT::Data::MaterialList;
using GVT::Data::ray;
using GVT::Math::Point4f;
using GVT::Math::Vector4f;

namespace GVT {

namespace Data {

Mesh::Mesh(Material* mat) : mat(NULL), haveNormals(false) {}

Mesh::Mesh(const Mesh& orig) {
  mat = orig.mat;
  vertices = orig.vertices;
  mapuv = orig.mapuv;
  normals = orig.normals;
  faces = orig.faces;
  faces_to_normals = orig.faces_to_normals;
  face_normals = orig.face_normals;
  faces_to_materials = orig.faces_to_materials;
  boundingBox = orig.boundingBox;
  haveNormals = orig.haveNormals;
}

Mesh::~Mesh() {}

void Mesh::addVertex(Point4f vertex, Vector4f normal, Point4f texUV) {
  vertices.push_back(vertex);
  normals.push_back(normal);
  mapuv.push_back(texUV);
  boundingBox.expand(vertex);
}

void Mesh::pushNormal(int which, Vector4f normal) {
  GVT_ASSERT((which > vertices.size()), "Adding normal outside the bounds");
  normals[which] = normal;
}

void Mesh::pushTexUV(int which, Point4f texUV) {
  GVT_ASSERT((which > vertices.size()), "Adding texture outside the bounds");
  this->mapuv[which] = texUV;
}

void Mesh::pushVertex(int which, Point4f vertex, Vector4f normal,
                      Point4f texUV) {
  GVT_ASSERT((which > vertices.size()), "Adding vertex outside the bounds");
  vertices[which] = vertex;
  boundingBox.expand(vertex);
  if (normal.length2()) normals[which] = normal;
  if (texUV.length2()) this->mapuv[which] = texUV;
}

void Mesh::setMaterial(Material* mat) { this->mat = mat; }

void Mesh::setMaterialList(MaterialList* materials) {
  material_list = materials;
}

void Mesh::addFace(int v0, int v1, int v2) {
  GVT_ASSERT(v0 < vertices.size(), "Vertex index 0 outside bounds");
  GVT_ASSERT(v1 < vertices.size(), "Vertex index 1 outside bounds");
  GVT_ASSERT(v2 < vertices.size(), "Vertex index 2 outside bounds");
  faces.push_back(face(v0, v1, v2));
}

void Mesh::setFaceMaterial(int face_id, const Material* material) {
  faces_to_materials[face_id] = material;
}

void Mesh::generateNormals() {
  if (haveNormals) return;
  std::cout << "Generating normals\n";
  normals.clear();
  face_normals.clear();
  faces_to_normals.clear();
  normals.resize(vertices.size());
  face_normals.resize(faces.size());
  faces_to_normals.resize(faces.size());
  for (int i = 0; i < normals.size(); ++i)
    normals[i] = Vector4f(0.0f, 0.0f, 0.0f, 0.0f);

  for (int i = 0; i < faces.size(); ++i) {
    int I = faces[i].get<0>();
    int J = faces[i].get<1>();
    int K = faces[i].get<2>();
    Vector4f a = vertices[I];
    Vector4f b = vertices[J];
    Vector4f c = vertices[K];
    Vector4f u = b - a;
    Vector4f v = c - a;
    Vector4f normal;
    normal.n[0] = u.n[1] * v.n[2] - u.n[2] * v.n[1];
    normal.n[1] = u.n[2] * v.n[0] - u.n[0] * v.n[2];
    normal.n[2] = u.n[0] * v.n[1] - u.n[1] * v.n[0];
    normal.n[3] = 0.0f;
    normal.normalize();
    face_normals[i] = normal;
    normals[I] += normal;
    normals[J] += normal;
    normals[K] += normal;
    faces_to_normals[i] = face_to_normals(I, J, K);
  }
  for (int i = 0; i < normals.size(); ++i) normals[i].normalize();
  //for (int i = 0; i < normals.size(); ++i)
  //  std::cout << "normal = (" << normals[i].n[0] <<
  //   "," << normals[i].n[1] << "," << normals[i].n[2] <<  ")\n";
  haveNormals = true;
}

Color Mesh::shadeFace(int face_id, ray& r, Vector4f normal,
                  LightSource* lsource) {
  Color c(0.5f, 0.5f, 0.5f, 0.0f);
  const Material* m =
      (faces_to_materials[face_id] ? faces_to_materials[face_id] : mat);
  if (m) {
    c = m->shade(r, normal, lsource);
  }
  return c;
}

Color Mesh::shade(ray& r, Vector4f normal, LightSource* lsource) {
  return mat->shade(r, normal, lsource);
}

}  // namespace Data

}  // namespace GVT

