/* =======================================================================================
   This file is released as part of GraviT - scalable, platform independent ray tracing
   tacc.github.io/GraviT

   Copyright 2013-2015 Texas Advanced Computing Center, The University of Texas at Austin
   All rights reserved.

   Licensed under the BSD 3-Clause License, (the "License"); you may not use this file
   except in compliance with the License.
   A copy of the License is included with this software in the file LICENSE.
   If your copy does not contain the License, you may obtain a copy of the License at:

       http://opensource.org/licenses/BSD-3-Clause

   Unless required by applicable law or agreed to in writing, software distributed under
   the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
   KIND, either express or implied.
   See the License for the specific language governing permissions and limitations under
   limitations under the License.

   GraviT is funded in part by the US National Science Foundation under awards ACI-1339863,
   ACI-1339881 and ACI-1339840
   ======================================================================================= */
/*
 * File:   Material.cuh
 * Author: Roberto Ribeiro
 *
 * Created on February 4, 2016, 19:00 PM
 */


#ifndef GVT_RENDER_DATA_PRIMITIVES_MATERIAL_CUH
#define GVT_RENDER_DATA_PRIMITIVES_MATERIAL_CUH

//#include <gvt/core/Math.h>
//#include <gvt/render/actor/Ray.h>
//#include <gvt/render/data/scene/Light.h>

//#include <boost/container/vector.hpp>
//#include <time.h>

#include <vector_functions.h>

#include "Ray.cuh"
#include "Light.cuh"

__device__ float cudaRand( );


namespace gvt {
namespace render {
namespace data {
namespace cuda_primitives {



typedef enum {BASE_MATERIAL, LAMBERT, PHONG, BLINN} MATERIAL_TYPE;




/// surface material properties
/** surface material properties used to shade intersected geometry
*/
class BaseMaterial {
public:
/*   __device__ BaseMaterial(){

  }
  // __device__ Material(const Material &orig);
   __device__  virtual ~BaseMaterial(){

  }*/

	// __device__ cuda_vec shade(const Ray &ray,                const cuda_vec &sufaceNormal,
      //                                    const Light *lightSource);
 /*  gvt::render::actor::RayVector ao(const gvt::render::actor::Ray &ray,
                                           const gvt::core::math::Vector4f &sufaceNormal, float samples);
   gvt::render::actor::RayVector secondary(const gvt::render::actor::Ray &ray,
                                                  const gvt::core::math::Vector4f &sufaceNormal, float samples);*/




   __device__  cuda_vec CosWeightedRandomHemisphereDirection2(cuda_vec n) ;


};

class Lambert : public BaseMaterial {
public:
/*   __device__ Lambert(cuda_vec kd = make_cuda_vec(0)){

  }
   __device__ virtual ~Lambert(){

  }*/

	 __device__ cuda_vec shade( const Ray &ray,
            const cuda_vec &sufaceNormal,
            const Light *lightSource);

                                          /*
  virtual gvt::render::actor::RayVector ao(const gvt::render::actor::Ray &ray,
                                           const gvt::core::math::Vector4f &sufaceNormal, float samples);
  virtual gvt::render::actor::RayVector secundary(const gvt::render::actor::Ray &ray,
                                                  const gvt::core::math::Vector4f &sufaceNormal, float samples);*/

  cuda_vec kd;
};



class Phong : public BaseMaterial {
public:

	/* __device__ Phong(const cuda_vec &kd = make_cuda_vec(0),
        const cuda_vec &ks = make_cuda_vec(0), const float &alpha = 1.f);

	 __device__ virtual ~Phong();
*/
	 __device__ cuda_vec shade(const Ray &ray,
                                          const cuda_vec &sufaceNormal,
                                          const Light *lightSource);

  /*
  virtual gvt::render::actor::RayVector ao(const gvt::render::actor::Ray &ray,
                                           const gvt::core::math::Vector4f &sufaceNormal, float samples);
  virtual gvt::render::actor::RayVector secundary(const gvt::render::actor::Ray &ray,
                                                  const gvt::core::math::Vector4f &sufaceNormal, float samples);*/

 cuda_vec kd;
 cuda_vec ks;
  float alpha;
};


class BlinnPhong : public BaseMaterial {
public:
 /* BlinnPhong(const gvt::core::math::Vector4f &kd = gvt::core::math::Vector4f(),
             const gvt::core::math::Vector4f &ks = gvt::core::math::Vector4f(), const float &alpha = 1.f);
  BlinnPhong(const BlinnPhong &orig);
  virtual ~BlinnPhong();
*/
	 __device__ cuda_vec shade(const Ray &ray,
                                          const cuda_vec &sufaceNormal,
                                          const Light *lightSource);

  /*
  virtual gvt::render::actor::RayVector ao(const gvt::render::actor::Ray &ray,
                                           const gvt::core::math::Vector4f &sufaceNormal, float samples);
  virtual gvt::render::actor::RayVector secundary(const gvt::render::actor::Ray &ray,
                                                  const gvt::core::math::Vector4f &sufaceNormal, float samples);*/

  cuda_vec kd;
  cuda_vec ks;
  float alpha;
};


typedef struct {

	MATERIAL_TYPE type;
	union {
		BaseMaterial material;
		Lambert lambert;
		Phong phong;
		BlinnPhong blinn;
	};

	 __device__
	cuda_vec shade(const Ray &ray, const cuda_vec &sufaceNormal,
			const Light *lightSource) {

		cuda_vec r;
		switch (type) {
		case BASE_MATERIAL:
			//r = material.shade(ray, sufaceNormal, lightSource);
			break;
		case LAMBERT:
			r = lambert.shade(ray, sufaceNormal, lightSource);
			break;
		case PHONG:
			r = phong.shade(ray, sufaceNormal, lightSource);
			break;
		case BLINN:
			r = blinn.shade(ray, sufaceNormal, lightSource);
			break;
		default:
			break;
		}
		return r;
	}

} Material;


}
}
}
}

#endif /* GVT_RENDER_DATA_PRIMITIVES_MATERIAL_CUH */
