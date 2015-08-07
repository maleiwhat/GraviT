//
//  TiledMantaRayTracer.h
//


#ifndef GVTAPPS_RENDER_TILED_MANTA_RAY_TRACER_H
#define GVTAPPS_RENDER_TILED_MANTA_RAY_TRACER_H

// #include <gvt/render/Attributes.h>
// #include <gvt/render/Context.h>
// #include <gvt/render/data/Domains.h>
// #include <gvt/render/data/Primitives.h>
// #include <gvt/render/data/scene/Image.h>
#include <apps/render/ConfigFileLoader.h>
#include <gvt/render/data/Dataset.h>

// #include <pthread.h>
// #include <semaphore.h>

// #include <algorithm>
// #include <set>
#include <string>

namespace gvtapps {
    namespace render {

        class TiledMantaRayTracer
        {
        public:
            TiledMantaRayTracer(gvtapps::render::ConfigFileLoader& cl);

            void RenderImage(std::string);
            gvt::render::actor::RayVector rays;
            gvt::render::data::Dataset *scene;
	    int m_width;
	    int m_height;
        };
    }
}

#endif // GVTAPPS_RENDER_TILED_MANTA_RAY_TRACER_H
