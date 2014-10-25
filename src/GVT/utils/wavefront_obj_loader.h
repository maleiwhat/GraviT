#ifndef WAVEFRONT_OBJ_LOADER_H
#define WAVEFRONT_OBJ_LOADER_H

#include <GVT/Data/primitives/gvt_material.h>
#include <GVT/Data/primitives/gvt_material_list.h>
#include <GVT/Data/primitives/gvt_mesh.h>
#include <objreader/usercallbacks.h>

namespace GVT {

namespace utils {

class WavefrontObjLoader {
 public:

  // The file path specifies the obj file to load and the "mesh"
  // parameter will be populated with the geometry from the path
  // specified by "file_path".
  WavefrontObjLoader(const std::string& file_path, GVT::Data::Mesh* mesh)
      : file_path_(file_path),
        real_path_(),
        parent_path_(),
        mesh_(mesh),
        current_face_vertex_(-1) {
    Init();
  }

  // This causes the reading and loading process to start.  The mesh
  // should contain the geometry from the .obj file as triangle soup.
  int Load();

  // This must add a vertex with coordinates "x", "y", "z", and "weight"
  // as the values and create and ID for the new vertex to be referenced
  // later on.  It there was no error, the value of 0 should be returned,
  // else a non-zero value can be returned.
  int AddVertex(float x, float y, float z, float weight);

  // This must add a texel with coordinates "x", "y", "z", and "weight"
  // as the values and create and ID for the new vertex to be referenced
  // later on.  It there was no error, the value of 0 should be returned,
  // else a non-zero value can be returned.
  int AddTexel(float x, float y, float z);

  // This must add a normal with coordinates "x", "y", and "z"
  // as the values and create an ID for the new vertex to be referenced
  // later on. It there was no error, the value of 0 should be returned,
  // else a non-zero value can be returned.
  int AddNormal(float x, float y, float z);

  // This must start a face and create an ID for it.  Must return 0
  // on error and non-zero otherwise.
  int StartFace();

  // Add a material library using "file_name" as the path.  The file name
  // should use the ".mtl" extension.
  int AddMaterialLib(const char* file_name);

  // Use the material with name specified by "material_name".
  int UseMaterial(const char* material_name);

  // This must add a vertex to the current face with vertex ID of "v",
  // vertex texel ID of "vt", and vertex normal with ID of "vn".
  // All vertices in a face must have a texel coordinate or none. The same
  // goes for vertex normals. Must return 0 on error and non-zero otherwise.
  //
  // NOTE: All faces are converted to triangles by interpreting the face as
  // a triangle fan if it has more than 3 vertices.
  //
  // NOTE: The vt parameter is currently unused.
  int AddToFace(size_t v, size_t vt, size_t vn);

 protected:
  // Currently, this method is disabled.
  WavefrontObjLoader(); 

  // This sets up callbacks for libobjreader.
  void Init();

 private:
  // This is the location of the .obj file to load.
  std::string file_path_;
  std::string real_path_;  // absolute location of the file
  std::string parent_path_;  // parent of real_path_
  // This is the mesh being loaded into.
  GVT::Data::Mesh* mesh_;
  // Since we triangulate all polygonal faces, we need to keep track
  // of how many vertices were loaded. This holds said value.
  int current_face_vertex_;
  // The callbacks struct provided by libobjreader.
  ObjParseCallbacks object_parse_callbacks_;
  // This is the current material ibrary.
  GVT::Data::MaterialLibrary* current_material_library_;
  // This is the current material.
  const GVT::Data::Material* current_material_;

  // Disable copy.
  WavefrontObjLoader(const WavefrontObjLoader&);
  // Disable assign.
  WavefrontObjLoader& operator=(const WavefrontObjLoader&);
};

}  // namespace utils

}  // namespace GVT
#endif  // WAVEFRONT_OBJ_LOADER_H

