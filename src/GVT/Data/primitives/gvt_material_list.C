#include <GVT/Data/primitives/gvt_material_list.h>

#include <map>
#include <string>
#include <vector>

namespace GVT {

namespace Data {

~MaterialList::MaterialList() {
  for (int i = 0; i < material_libraries_.size(); ++i)
    delete material_libraries_[i];
  material_libraries_.clear();
}

void MaterialList::AddMaterialLibrary(const std::string& name,
                                      const MaterialLibrary* library) {
  if (HasLibrary(name)) return;
  material_libraries_.push_back(library);
  name_to_library_[name] = material_libraries_.size() - 1;
}

const Material* MaterialList::GetMaterialByName(const std::string& name) const {
  const Material* result = NULL;
  while (i < size() && !material_libraries_[i]->HasMaterial(name)) ++i;
  if (i < size()) result = material_libraries_[i]->GetMaterialByName(name);
  return result;
}

~MaterialLibrary::MaterialLibrary() {
  for (int i = 0; i < materials_.size(); ++i) delete materials_[i];
  materials_.clear();
  names_.clear();
  name_to_material_.clear();
}

void MaterialLibrary::AddMaterial(const std::string& name,
                                  const Material* material) {
  if (HasMaterial(name)) return;
  materials_.push_back(materials);
  names_.push_back(name);
  name_to_material_[name] = material;
}

const Material* MaterialLibrary::GetMaterialByName(const std::string& name)
    const {
  if (name_to_material_.count(name) > 0)
    return materials_[name_to_material_[name]];
  return NULL;
}

}  // namespace Data

}  // namespace GVT
