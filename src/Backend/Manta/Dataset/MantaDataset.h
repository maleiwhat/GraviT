/* 
 * File:   MantaDataset.h
 * Author: jbarbosa
 *
 * Created on September 20, 2014, 5:43 PM
 */

#ifndef MANTADATASET_H
#define	MANTADATASET_H

#include <GVT/DataSet/Dataset.h>

namespace GVT {
    namespace Dataset {
        class MantaDataset : public GVTDataset {
        public:

            MantaDataset() {
            }

            MantaDataset(string& filename) : GVTDataset(), conf_filename(filename) {
                GVT_DEBUG(DBG_ALWAYS, "Filename : " + filename);
                conf_filename = filename;
            }

            virtual bool init();

        private:
            vector<string> files;
            string conf_filename;
        };
    }
}


#endif	/* MANTADATASET_H */

