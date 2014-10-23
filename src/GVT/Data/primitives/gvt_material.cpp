/*
 * File:   gvt_material.cpp
 * Author: jbarbosa
 *
 * Created on April 18, 2014, 3:07 PM
 */
#include <cmath>

#include <GVT/Data/derived_types.h>
#include <GVT/Data/primitives/gvt_material.h>

using GVT::Data::Color;
using GVT::Data::LightSource;
using GVT::Data::RayVector;
using GVT::Data::ray;
using GVT::Math::Vector4f;

namespace GVT {

namespace Data {

Material::Material() {}

Material::Material(const Material& orig) {}

Material::~Material() {}

Vector4f Material::shade(const ray& ray, const Vector4f& sufaceNormal,
                         const LightSource* lightSource) const {
  return Vector4f();
}

RayVector Material::ao(const ray& ray, const Vector4f& sufaceNormal,
                       float samples) const {
  return RayVector();
}

RayVector Material::secondary(const ray& ray, const Vector4f& sufaceNormal,
                              float samples) const {
  return RayVector();
}

Lambert::Lambert(const Vector4f& kd) : Material(), kd(kd) {}

Lambert::Lambert(const Lambert& orig) : Material(orig), kd(orig.kd) {}

Lambert::~Lambert() {}

Vector4f Lambert::shade(const ray& ray, const Vector4f& N,
                        const LightSource* lightSource) const {

  Point4f L = ray.direction;
  L = L.normalize();
  float NdotL = std::max(0.f, N * L);
  Color lightSourceContrib = lightSource->contribution(ray);
  Color diffuse = prod(lightSourceContrib, kd * NdotL) * ray.w;
  return diffuse;
}

RayVector Lambert::ao(const ray& ray, const Vector4f& sufaceNormal,
                      float samples) const {
  return RayVector();
}

RayVector Lambert::secondary(const ray& ray, const Vector4f& sufaceNormal,
                             float samples) const {
  return RayVector();
}

Phong::Phong(const Vector4f& kd, const Vector4f& ks, const float& alpha)
    : Material(), kd(kd), ks(ks), alpha(alpha) {}

Phong::Phong(const Phong& orig)
    : Material(orig), kd(orig.kd), ks(orig.ks), alpha(orig.alpha) {}

Phong::~Phong() {}

Vector4f Phong::shade(const ray& ray, const Vector4f& N,
                      const LightSource* lightSource) const {
  Vector4f hitPoint = (Vector4f)ray.origin + (ray.direction * ray.t);
  Vector4f L = (Vector4f)lightSource->position - hitPoint;

  L = L.normalize();
  float NdotL = std::max(0.f, (N * L));
  Vector4f R = ((N * 2.f) * NdotL) - L;
  float VdotR = std::max(0.f, (R * (-ray.direction)));
  float power = VdotR * std::pow(VdotR, alpha);

  Vector4f lightSourceContrib = lightSource->contribution(ray);  //  distance;

  Color diffuse = prod((lightSourceContrib * NdotL), kd) * ray.w;
  Color specular = prod((lightSourceContrib * power), ks) * ray.w;

  Color finalColor = (diffuse + specular);
  return finalColor;
}

RayVector Phong::ao(const ray& ray, const Vector4f& sufaceNormal,
                    float samples) const {
  return RayVector();
}

RayVector Phong::secondary(const ray& ray, const Vector4f& sufaceNormal,
                           float samples) {
  return RayVector();
}

BlinnPhong::BlinnPhong(const Vector4f& kd, const Vector4f& ks,
                       const float& alpha)
    : Material(), kd(kd), ks(ks), alpha(alpha) {}

BlinnPhong::BlinnPhong(const BlinnPhong& orig)
    : Material(orig), kd(orig.kd), ks(orig.ks), alpha(orig.alpha) {}

BlinnPhong::~BlinnPhong() {}

Vector4f BlinnPhong::shade(const ray& ray, const Vector4f& N,
                           const LightSource* lightSource) const {
  Vector4f hitPoint = (Vector4f)ray.origin + (ray.direction * ray.t);
  Vector4f L = (Vector4f)lightSource->position - hitPoint;
  L = L.normalize();
  float NdotL = std::max(0.f, (N * L));

  Vector4f H = (L - ray.direction).normalize();

  float NdotH = (H * N);
  float power = NdotH * std::pow(NdotH, alpha);

  Vector4f lightSourceContrib = lightSource->contribution(ray);

  Color diffuse = prod((lightSourceContrib * NdotL), kd) * ray.w;
  Color specular = prod((lightSourceContrib * power), ks) * ray.w;

  Color finalColor = (diffuse + specular);
  return finalColor;
}

RayVector BlinnPhong::ao(const ray& ray, const Vector4f& sufaceNormal,
                         float samples) const {
  return RayVector();
}

RayVector BlinnPhong::secondary(const ray& ray, const Vector4f& sufaceNormal,
                                float samples) const {
  return RayVector();
}

WavefrontObjMaterial::WavefrontObjMaterial()
    : kd(Vector4f(0.5f, 0.5f, 0.5f, 0.0f)),
      ks(Vector4f(0.0f, 0.0f, 0.0f, 0.0f)),
      ke(Vector4f(0.0f, 0.0f, 0.0f, 0.0f)),
      ka(Vector4f(0.0f, 0.0f, 0.0f, 0.0f)),
      specular_exponent(0.0f),
      has_illum_model(false),
      illum_model(0),
      has_ambient_texture_map(false),
      ambient_texture_map(),
      has_diffuse_texture_map(false),
      diffuse_texture_map() {}

WavefrontObjMaterial::WavefrontObjMaterial(const WavefrontObjMaterial& orig)
    : kd(orig.kd),
      ks(orig.ks),
      ke(orig.ke),
      ka(orig.ka),
      specular_exponent(orig.specular_exponent),
      has_illum_model(orig.has_illum_model),
      illum_model(orig.illum_model),
      has_ambient_texture_map(orig.has_ambient_texture_map),
      ambient_texture_map(orig.ambient_texture_map),
      has_diffuse_texture_map(orig.has_diffuse_texture_map),
      diffuse_texture_map(orig.diffuse_texture_map) {}

WavefrontObjMaterial::~WavefrontObjMaterial() {}

Vector4f WavefrontObjMaterial::shade(const ray& ray, const Vector4f& N,
                                     const LightSource* lightSource) const {
  Vector4f hitPoint = (Vector4f)ray.origin + (ray.direction * ray.t);
  Vector4f L = (Vector4f)lightSource->position - hitPoint;
  L = L.normalize();
  float NdotL = std::max(0.f, (N * L));

  Vector4f H = (L - ray.direction).normalize();

  float NdotH = (H * N);
  float power = NdotH * std::pow(NdotH, specular_exponent);

  Vector4f lightSourceContrib = lightSource->contribution(ray);

  Color diffuse = prod((lightSourceContrib * NdotL), kd) * ray.w;
  Color specular = prod((lightSourceContrib * specular_exponent), ks) * ray.w;

  // TODO (rsmith): Add support for texture maps and illumination models.
  // However, diffuse texture maps are the highest priority.
  //
  // TODO (rsmith): Add support for emissive and ambient properties.
  // Emissive property handling is the highest priority.

  Color finalColor = (diffuse + specular);
  return finalColor;
}

RayVector WavefrontObjMaterial::ao(const ray& ray, const Vector4f& sufaceNormal,
                                   float samples) const {
  return RayVector();
}

RayVector WavefrontObjMaterial::secondary(const ray& ray,
                                          const Vector4f& sufaceNormal,
                                          float samples) const {
  return RayVector();
}

}  // namespace Data

}  // namespace GVT
