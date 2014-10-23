#include <GVT/utils/wavefront_mtl_loader.h>

#include <cstdio>
#include <cstdlib>

#include <GVT/Data/primitives.h>
#include <GVT/Math/GVTMath.h>
#include <objreader/objreader.h>
#include <objreader/usercallbacks.h>

using GVT::Data::Mesh;
using GVT::Math::Point4f;
using GVT::Math::Vector4f;

namespace GVT {

namespace utils {

// Material Parse Callbacks
//
// These are provided as a bridge between C and C++.
// User data should be of type WavefrontMtlLoader*.  The function will
// cast user_data to said pointer type and call the corresponding method.
int AddMaterialCallback(char* name, void* user_data);
int SetAmbientColorCallback(float r, float g, float b, void* user_data);
int SetDiffuseColorCallback(float r, float g, float b, void* user_data);
int SetSpecularColorCallback(float r, float g, float b, void* user_data);
int SetSpecularExponentCallback(float se, void* user_data);
int SetOpticalDensityCallback(float d, void* user_data);
int SetAlphaCallback(float a, void* user_data);
int SetIllumModelCallback(int model, void* user_data);
int SetAmbientTextureMapCallback(char* path, void* user_data);
int SetDiffuseTextureMapCallback(char* path, void* user_data);

WavefrontMtlLoader::WavefrontMtlLoader() { Init(); }

void WavefrontMtlLoader::Init() {
  // Setup object parse callbacks.
  memset(&material_parse_callbacks_, 0, sizeof(ObjParseCallbacks));
  material_parse_callbacks_.onAddMaterial = AddMaterial;
  material_parse_callbacks_.onSetAmbientColor = SetAmbientColor;
  material_parse_callbacks_.onSetDiffuseColor = SetDiffuseColor;
  material_parse_callbacks_.onSetSpecularColor = SetSpecularColor;
  material_parse_callbacks_.onSetSpecularExponent = SetSpecularExponent;
  material_parse_callbacks_.onSetOpticalDensity = SetOpticalDensity;
  material_parse_callbacks_.onSetAlpha = SetAlpha;
  material_parse_callbacks_.onSetIllumModel = SetIllumModel;
  material_parse_callbacks_.onSetAmbientTextureMap = SetAmbientTextureMap;
  material_parse_callbacks_.onSetDiffuseTextureMap = SetDiffuseTextureMap;
  material_parse_callbacks_.userData = reinterpret_cast<void*>(this);
}

int WavefrontMtlLoader::AddMaterial(char* name) {
  current_material_  = new ObjMaterial;
  material_library_->AddMaterial(name,
                                 static_cast<Material*>(current_material_));
}

int WavefrontMtlLoader::SetAmbientColor(float r, float g, float b) {
  if (current_material_) {
    current_material_->set_ka(Vector4f(r, g, b, 0.0f));
  }
  return 0;
}

int WavefrontMtlLoader::SetDiffuseColor(float r, float g, float b) {
  if (current_material_) {
    current_material_->set_kd(Vector4f(r, g, b, 0.0f));
  }
  return 0;
}

int WavefrontMtlLoader::SetSpecularColor(float r, float g, float b) {
  if (current_material_) {
    current_material_->set_ks(Vector4f(r, g, b, 0.0f));
  }
  return 0;
}

int WavefrontMtlLoader::SetSpecularExponent(float se) {
  if (current_material_) {
    current_material_->set_specular_exponent(se);
  }
  return 0;
}

int WavefrontMtlLoader::SetOpticalDensity(float d) {
  if (current_material_) {
    current_material_->set_optical_density(d);
  }
  return 0;
}

int WavefrontMtlLoader::SetAlpha(float a) {
  if (current_material_) {
    current_material_->set_alpha(a);
  }
  return 0;
}

int WavefrontMtlLoader::SetIllumModel(int model) {
  if (current_material_) {
    current_material_->set_has_illum_model(true);
    current_material_->set_illum_model(model);
  }
  return 0;
}

int WavefrontMtlLoader::SetAmbientTextureMap(char* path) {
  if (current_material_) {
    current_material_->set_has_ambient_texture_map(true);
    current_material_->set_ambient_texture_map(std::string(path));
  }
  return 0;
}

int WavefrontMtlLoader::SetDiffuseTextureMap(char* path) {
  if (current_material_) {
    current_material_->set_has_diffuse_texture_map(true);
    current_material_->set_diffuse_texture_map(std::string(path));
  }
  return 0;
}


// Class methods used by WavefrontMtlLoader here.
// We provide a very simple implementation to load faces, vertices,
// and vertex normals.
int WavefrontMtlLoader::Load() {
  FILE* file_stream = fopen(file_path_.c_str(), "r");
  if (!file_stream) std::cout << "Unable to open: " << file_path_ << "\n";
  std::cout << "Loading " << file_path_ << "\n";
  int result = ReadMtlFile(
      file_stream, const_cast<MtlParseCallbacks*>(&material_parse_callbacks_));
  fclose(file_stream);
  return result;
}

int AddMaterialCallback(char* name, void* user_data) {
  WavefrontMtlLoader* loader = reinterpret_cast<WavefrontMtlLoader*>(user_data);
  int result = loader->AddMaterial(name);
  return result;
}

int SetAmbientColorCallback(float r, float g, float b, void* user_data) {
  WavefrontMtlLoader* loader = reinterpret_cast<WavefrontMtlLoader*>(user_data);
  int result = loader->SetAmbientColor(r, g, b);
  return result;
}

int SetDiffuseColorCallback(float r, float g, float b, void* user_data) {
  WavefrontMtlLoader* loader = reinterpret_cast<WavefrontMtlLoader*>(user_data);
  int result = loader->SetDiffuseColor(r, g, b);
  return result;
}

int SetSpecularColorCallback(float r, float g, float b, void* user_data) {
  WavefrontMtlLoader* loader = reinterpret_cast<WavefrontMtlLoader*>(user_data);
  int result = loader->SetSpecularColor(r, g, b);
  return result;
}

int SetSpecularExponentCallback(float se, void* user_data) {
  WavefrontMtlLoader* loader = reinterpret_cast<WavefrontMtlLoader*>(user_data);
  int result = loader->SetSpecularExponent(se);
  return result;
}

int SetOpticalDensityCallback(float d, void* user_data) {
  WavefrontMtlLoader* loader = reinterpret_cast<WavefrontMtlLoader*>(user_data);
  int result = loader->SetOpticalDensity(d);
  return result;
}

int SetAlphaCallback(float a, void* user_data) {
  WavefrontMtlLoader* loader = reinterpret_cast<WavefrontMtlLoader*>(user_data);
  int result = loader->SetAlpha(a);
  return result;
}

int SetIllumModelCallback(int model, void* user_data) {
  WavefrontMtlLoader* loader = reinterpret_cast<WavefrontMtlLoader*>(user_data);
  int result = loader->SetIllumModel(model);
  return result;
}

int SetAmbientTextureMapCallback(char* path, void* user_data) {
  WavefrontMtlLoader* loader = reinterpret_cast<WavefrontMtlLoader*>(user_data);
  int result = loader->SetAmbientTextureMap(path);
  return result;
}

int SetDiffuseTextureMapCallback(char* path, void* user_data) {
  WavefrontMtlLoader* loader = reinterpret_cast<WavefrontMtlLoader*>(user_data);
  int result = loader->SetDiffuseTextureMap(path);
  return result;
}

}  // namespace utils

}  // namespace GVT
