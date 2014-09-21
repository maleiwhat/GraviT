//
// Dataset.h
//

#ifndef GVT_CF_DATASET_H
#define GVT_CF_DATASET_H

#include <cfloat>
#include <map>
#include <string>
#include <vector>

//#include <Backend/Manta/Domain/MantaDomain.h>
//#include <Backend/Optix/Domain/OptixDomain.h>
#include <GVT/Data/primitives.h>
#include <GVT/DataSet/Dataset.h>
#include <GVT/Domain/domains.h>
#include <GVT/common/debug.h>

using namespace std;

namespace GVT {
namespace Dataset {

template <typename DomainType>
class ConfigFileDataset : public GVTDataset {
 public:
  ConfigFileDataset() {}

  ConfigFileDataset(string& filename) : GVTDataset(), conf_filename(filename) {
    GVT_DEBUG(DBG_ALWAYS, "Filename : " + filename);
    conf_filename = filename;
  }

  virtual bool init() {
    GVT_DEBUG(DBG_ALWAYS, "Generic load");
    return false;
  }

 private:
  vector<string> files;
  string conf_filename;
};

template <>
bool ConfigFileDataset<GVT::Domain::VolumeDomain>::init();

template <>
bool ConfigFileDataset<GVT::Domain::GeometryDomain>::init();
//
//
//
//template <>
//bool Dataset<GVT::Domain::OptixDomain>::init();

}  // namespace Dataset

}  // namespave GVT

#endif  // GVT_DATASET_H
