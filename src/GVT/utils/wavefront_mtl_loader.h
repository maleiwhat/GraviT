#ifndef WAVEFRONT_MTL_LOADER_H
#define WAVEFRONT_MTL_LOADER_H

#include <GVT/Data/primitives/gvt_mesh.h>
#include <objreader/usercallbacks.h>

namespace GVT {

namespace utils {

class WavefrontMtlLoader {
 public:

  // The file path specifies the obj file to load and the "mesh"
  // parameter will be populated with the geometry from the path
  // specified by "file_path".
  WavefrontMtlLoader(const std::string& file_path,
                     GVT::Data::MaterialLibrary* material_library)
      : file_path_(file_path),
        material_library_(material_library),
        current_material_(NULL) {
    Init();
  }

  // This causes the reading and loading process to start.  The mesh
  // should contain the geometry from the .obj file as triangle soup.
  int Load();

  // This adds the material and associates it with the name of "name".
  // This remains the current material until another material is added.
  int AddMaterial(char* name);

  // This sets the ambient color for the current material. The "r",
  // "g", "b" parameters represent the red, blue, and green channels
  // of the image, respectively. Affects only the current material.
  int SetAmbientColor(float r, float g, float b);

  // This sets the diffuse color for the current material. The "r",
  // "g", "b" parameters represent the red, blue, and green channels
  // of the image, respectively. Affects only the current material.
  int SetDiffuseColor(float r, float g, float b);

  // This sets the specular color for the current material. The "r",
  // "g", "b" parameters represent the red, blue, and green channels
  // of the imagei, respectively. Affects only the current material.
  int SetSpecularColor(float r, float g, float b);

  // The parameter "se" represents the Blinn-Phong shininess parameter.
  // Affects only the current material.
  int SetSpecularExponent(float se);

  // The "d" parameter represents the index of refraction.
  // Affects only the current material.
  int SetOpticalDensity(float d);

  // The "a" parameter represents the opacity.
  // Affects only the current material.
  int SetAlpha(float a);

  // Set the illumination model with parameter "model".
  // See http://paulbourke.net/dataformats/mtl/.
  int SetIllumModel(int model);

  // Provide a texture map for the ambient lighting using the file
  // specified by "path".
  int SetAmbientTextureMap(char* path);

  // Provide a texture map for the diffuse lighting using the file
  // specified by "path".
  int SetDiffuseTextureMap(char* path);

 protected:
  // Currently, this method is disabled.
  WavefrontMtlLoader(); 

  // This sets up callbacks for libobjreader.
  void Init();

 private:
  // This is the location of the .mtl file to load.
  std::string file_path_;
  // This is the mesh being loaded into.
  const GVT::Data::MaterialLibrary* material_library_;
  // The current material.
  const GVT::Data::Material* current_material_;
  // The callbacks struct provided by libobjreader.
  ObjParseCallbacks object_parse_callbacks_;

  // Disable copy.
  WavefrontMtlLoader(const WavefrontMtlLoader&);
  // Disable assign.
  WavefrontMtlLoader& operator=(const WavefrontMtlLoader&);
};

}  // namespace utils

}  // namespace GVT
#endif  // WAVEFRONT_OBJ_LOADER_H

