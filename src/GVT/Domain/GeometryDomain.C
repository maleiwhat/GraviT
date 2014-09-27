//
// GeometryDomain.C
//
#include <GVT/Domain/GeometryDomain.h>

#include <cctype>

#include <GVT/utils/readply.h>
#include <GVT/utils/wavefront_obj_loader.h>

using GVT::utils::WavefrontObjLoader;

namespace GVT {

namespace Domain {

std::string ToLower(const std::string& str) {
  std::string result(str);
  for (int i = 0; i < str.length(); ++i) result[i] = tolower(result[i]);
  return result;
}

std::string GetFileExtension(const std::string& path) {
  std::string name = path.substr(path.find_last_of("/") + 1);
  std::string ext = name.substr(name.find_last_of(".") + 1);
  if (ext.length() == name.length()) return "";
  return ToLower(ext.erase(ext.find_last_not_of(" \t\f\v\n\r") + 1));
}

void GeometryDomain::free() {
  if (!isLoaded) return;
  for (int i = lights.size() - 1; i >= 0; i--) {
    delete lights[i];
    lights.pop_back();
  }
  if (mesh->mat) {
    delete mesh->mat;
    mesh->mat = NULL;
  }
  if (mesh) {
    delete mesh;
    mesh = NULL;
  }
  isLoaded = false;
}

//        bool GeometryDomain::intersect(GVT::Data::ray& ray, vector<int>&
// intersections) {
//            if (!boundingBox.intersect(ray)) {
//                intersections.clear();
//                return false;
//            } else {
//                return true;
//            }
//        }
//
bool GeometryDomain::load() {
  if (isLoaded) return true;
  if (filename == "") return false;
  std::string ext;
  if (ext.compare("ply") == 0) {
    mesh = readply(filename);
  } else if (ext.compare("obj") == 0) {
    WavefrontObjLoader loader(filename, mesh);
    loader.Load();
  }
  lights.push_back(
      new GVT::Data::PointLightSource(GVT::Math::Point4f(5.0, 5.0, 5.0, 1.f),
                                      GVT::Data::Color(1.f, 1.f, 1.f, 1.f)));
  mesh->mat = new GVT::Data::Lambert(GVT::Data::Color(1.f, .0f, .0f, 1.f));
  boundingBox = mesh->boundingBox;
  isLoaded = true;
  return isLoaded;
}

ostream& operator<<(ostream& os, GeometryDomain const& d) {
  os << "geometry domain @ addr " << (void*)&d << endl;
  os << "    XXX not yet implemented XXX" << endl;
  os << flush;
  return os;
}

}  // namespace Domain

}  // namespace GVT
