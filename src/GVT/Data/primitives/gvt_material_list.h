#ifndef GVT_MATERIAL_LIST_H
#define GVT_MATERIAL_LIST_H

#include <map>
#include <string>
#include <vector>

#include <GVT/Data/primitives/gvt_material.h>

namespace GVT {

namespace Data {

  class MaterialLibrary {
    public:
      MaterialLibrary() : materials_(), names_(), name_to_material_() {};
      ~MaterialLibrary();
      // Add the material specified by the "name" parameter and pointer
      // to the material specified by the "material" parameter.  Takes
      // ownership of said pointer.
      void AddMaterial(const std::string& name, const Material* material);
      bool HasMaterial(const std::string& name) const {
        return (name_to_material_.count(name) > 0);
      }
      const GVT::Data::Material* GetMaterialByName(const std::string& name)
          const;
      std::string GetMaterialName(int i) const { return names_[i]; }
      const Material* operator[](int i) const {
        return (i < size() && i >= 0 ? materials_[i] : NULL);
      }
      int size() const { materials_.size(); }
    private:
      std::vector<const GVT::Data::Material*> materials_;
      std::vector<std::string> names_;
      std::map<std::string, int> name_to_material_;
  };

  class MaterialList {
    public:
        MaterialList() : material_libraries_() {}
        ~MaterialList();
        // Add the material specified by the "library" parameter.
        // Takes ownership of the pointer.
        void AddMaterialLibrary(const std::string& name,
                                const MaterialLibrary* library);
        bool HasLibrary(const std::string& name) const {
          return (name_to_library_.count(name) > 0);
        }
        const MaterialLibrary* operator[](int i) const {
          return (i < size() && i >= 0 ? material_libraries_[i] : NULL);
        }
        const GVT::Data::Material* GetMaterialByName(const std::string& name) const;
        int size() const { material_libraries_.size(); }

       private:
        std::vector<const MaterialLibrary*> material_libraries_;
        std::map<std::string, int> name_to_library_;
  };

}  // namespace Data

}  // namespace GVT

#endif  // GVT_MATERIAL_LIST_H
