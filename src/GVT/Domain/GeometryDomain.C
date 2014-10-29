//
// GeometryDomain.C
//
#include <GVT/Domain/GeometryDomain.h>

#include <cctype>

#include <GVT/Data/primitives/gvt_material_list.h>
#include <GVT/utils/readply.h>
#include <GVT/utils/wavefront_obj_loader.h>

using GVT::Data::MaterialList;
using GVT::utils::WavefrontObjLoader;

namespace GVT {

namespace Domain {

static std::string ToLower(const std::string& str) {
  std::string result(str);
  for (int i = 0; i < str.length(); ++i) result[i] = tolower(result[i]);
  return result;
}

static std::string GetFileExtension(const std::string& path) {
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
  if (mesh->material_list) {
    delete mesh->material_list;
    mesh->material_list = NULL;
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
  std::string ext = GetFileExtension(filename);;
  std::cout << "File extension = " << ext << "\n";
  if (ext.compare("ply") == 0) {
    mesh = readply(filename);
  } else if (ext.compare("obj") == 0) {
    std::cout << "Loading OBJ file.\n";
    mesh =  new GVT::Data::Mesh();
    mesh->material_list = new MaterialList;
    WavefrontObjLoader loader(filename, mesh);
    loader.Load();
   // std::cout << "Material list size = " << mesh->material_list->size() << "\n";
   // std::cout << "faces_to_materials.size() = "
    //          << mesh->faces_to_materials.size() << "\n";
    //for (int i = 0; i < mesh->faces.size(); ++i) {
    //  if (mesh->faces_to_materials.size() > i  &&
    //      mesh->faces_to_materials[i] != NULL) 
    //    std::cout << "face " << i << " has material \n" <<
   //     *(mesh->faces_to_materials[i]);
   // }
  }
  lights.push_back(new GVT::Data::PointLightSource(
  //    GVT::Math::Point4f(-10.7389f, -13.6002f, 1.0f),
//      GVT::Math::Point4f(-10.7098f, -13.9444f, 0.299326f), // sibenik cathedral
     // GVT::Math::Point4f(-20.0f, -13.0f, 1.0f),
        GVT::Math::Point4f(278.0f,400.4f,250.0f),   // cornell box
   //  GVT::Math::Point4f(0.0f, -4.0f, -4.0f),
  //    GVT::Math::Point4f(5.0f, 5.0f, 5.0f), // bunny
      GVT::Data::Color(1.f, 1.f, 1.f, 1.f)));
  mesh->mat = new GVT::Data::Lambert(GVT::Data::Color(1.f, .0f, .0f, 1.f));
  boundingBox = mesh->boundingBox;
  std::cout << "bounds = " << boundingBox << "\n";
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
