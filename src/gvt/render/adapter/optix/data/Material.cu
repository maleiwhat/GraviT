

#include "cutil_math.h"
#include "Material.cuh"


using namespace gvt::render::data::cuda_primitives;





   __device__  cuda_vec BaseMaterial::CosWeightedRandomHemisphereDirection2(cuda_vec n) {

    float Xi1 = cudaRand();
    float Xi2 = cudaRand();

    float theta = acos(sqrt(1.0 - Xi1));
    float phi = 2.0 * 3.1415926535897932384626433832795 * Xi2;

    float xs = sinf(theta) * cosf(phi);
    float ys = cosf(theta);
    float zs = sinf(theta) * sinf(phi);

    float3 y = make_cuda_vec(n);
    float3 h = y;
    if (fabs(h.x) <= fabs(h.y) && fabs(h.x) <= fabs(h.z))
      h.x = 1.0;
    else if (fabs(h.y) <= fabs(h.x) && fabs(h.y) <= fabs(h.z))
      h.y = 1.0;
    else
      h.z = 1.0;

    float3 x = cross(h,y);//(h ^ y);
    float3 z = cross(x, y);

    cuda_vec direction = make_cuda_vec(x * xs + y * ys + z * zs);
    return normalize(direction);
  }



 /* Material::Material() {}s

  Material::Material(const Material &orig) {}

  Material::~Material() {}
*/
/*
  cuda_vec BaseMaterial::shade(const Ray &ray, const cuda_vec &sufaceNormal, const Light *lightSource) {
	  return make_cuda_vec(0.f);
  }
*/

  /*RayVector Material::ao(const Ray &ray, const cuda_vec &sufaceNormal, float samples) { return RayVector(); }

  RayVector Material::secondary(const Ray &ray, const cuda_vec &sufaceNormal, float samples) { return RayVector(); }

  Lambert::Lambert(const cuda_vec &kd) : Material(), kd(kd) {}

  Lambert::Lambert(const Lambert &orig) : Material(orig), kd(orig.kd) {}

  Lambert::~Lambert() {}
*/
   __device__ cuda_vec Lambert::shade( const Ray &ray, const cuda_vec &N, const Light *lightSource) {


	   cuda_vec hitPoint = ray.origin + ray.direction * ray.t;
	   cuda_vec L = normalize(lightSource->light.position - hitPoint);
    float NdotL = fmaxf(0.f, (N * L));
    cuda_vec lightSourceContrib = lightSource->contribution(hitPoint);

    cuda_vec diffuse = prod(lightSourceContrib, kd) * (NdotL * ray.w);

    return diffuse;
  }
/*
  RayVector Lambert::ao(const Ray &ray, const cuda_vec &sufaceNormal, float samples) { return RayVector(); }

  RayVector Lambert::secundary(const Ray &ray, const cuda_vec &sufaceNormal, float samples) { return RayVector(); }

  Phong::Phong(const cuda_vec &kd, const cuda_vec &ks, const float &alpha) : Material(), kd(kd), ks(ks), alpha(alpha) {}

  Phong::Phong(const Phong &orig) : Material(orig), kd(orig.kd), ks(orig.ks), alpha(orig.alpha) {}

  Phong::~Phong() {}

  */

   __device__ cuda_vec Phong::shade(const Ray &ray, const cuda_vec &N, const Light *lightSource) {


   cuda_vec hitPoint = ray.origin + (ray.direction * ray.t);
   cuda_vec L =normalize(lightSource->light.position - hitPoint);

    float NdotL =fmaxf(0.f, (N * L));
    cuda_vec R = ((N * 2.f) * NdotL) - L;
    cuda_vec invDir = make_cuda_vec(-ray.direction.x, -ray.direction.y, -ray.direction.z);
    float VdotR = fmaxf(0.f, (R * invDir));
    float power = VdotR * pow(VdotR, alpha);

    cuda_vec lightSourceContrib = lightSource->contribution(hitPoint); //  distance;

    Color finalColor = prod(lightSourceContrib , kd) * (NdotL * ray.w);
    finalColor += prod(lightSourceContrib , ks) * (power * ray.w);
    return finalColor;


    return finalColor;
  }

  /*

  RayVector Phong::ao(const Ray &ray, const cuda_vec &sufaceNormal, float samples) { return RayVector(); }

  RayVector Phong::secundary(const Ray &ray, const cuda_vec &sufaceNormal, float samples) { return RayVector(); }

  BlinnPhong::BlinnPhong(const cuda_vec &kd, const cuda_vec &ks, const float &alpha)
      : Material(), kd(kd), ks(ks), alpha(alpha) {}

  BlinnPhong::BlinnPhong(const BlinnPhong &orig) : Material(orig), kd(orig.kd), ks(orig.ks), alpha(orig.alpha) {}

  BlinnPhong::~BlinnPhong() {}
*/
   __device__ cuda_vec BlinnPhong::shade(const Ray &ray, const cuda_vec &N, const Light *lightSource) {
//    cuda_vec hitPoint = (cuda_vec)ray.origin + (ray.direction * ray.t);
//    cuda_vec L = (cuda_vec)lightSource->light.position - hitPoint;
//    L = normalize(L);
//    float NdotL = fmaxf(0.f, (N * L));
//
//    cuda_vec H = normalize((L - ray.direction));
//
//    float NdotH = (H * N);
//    float power = NdotH * std::pow(NdotH, alpha);
//
//    cuda_vec lightSourceContrib = lightSource->contribution(ray);
//
//    Color diffuse = prod((lightSourceContrib * NdotL), kd) * ray.w;
//    Color specular = prod((lightSourceContrib * power), ks) * ray.w;
//
//    Color finalColor = (diffuse + specular);
//    return finalColor;

	   cuda_vec hitPoint = ray.origin + (ray.direction * ray.t);
	   cuda_vec L = normalize(lightSource->light.position - hitPoint);
	   float NdotL = fmaxf(0.f, (N* L));

	   cuda_vec H = normalize(L - ray.direction);

	   float NdotH = fmaxf(0.f, (H * N));
	   float power = NdotH * pow(NdotH, alpha);

	   cuda_vec lightSourceContrib = lightSource->contribution(hitPoint);

	   Color diffuse = prod(lightSourceContrib , kd) * (NdotL * ray.w);
	   Color specular = prod(lightSourceContrib , ks) * (power * ray.w);

	   Color finalColor = (diffuse + specular);
	   return finalColor;
  }
/*
  RayVector BlinnPhong::ao(const Ray &ray, const cuda_vec &sufaceNormal, float samples) { return RayVector(); }

  RayVector BlinnPhong::secundary(const Ray &ray, const cuda_vec &sufaceNormal, float samples) { return RayVector(); }*/
