/*
 * File:   gvt_material.h
 * Author: jbarbosa
 *
 * Created on April 18, 2014, 3:07 PM
 */

#ifndef GVT_MATERIAL_H
#define GVT_MATERIAL_H

#include <time.h>

#include <GVT/Data/primitives/gvt_lightsource.h>
#include <GVT/Data/primitives/gvt_ray.h>
#include <GVT/Math/GVTMath.h>
#include <boost/container/vector.hpp>

namespace GVT {

namespace Data {

class Material {
 public:
  Material();
  Material(const Material& orig);
  virtual ~Material();

  virtual GVT::Math::Vector4f shade(
      const GVT::Data::ray& ray, const GVT::Math::Vector4f& sufaceNormal,
      const GVT::Data::LightSource* lightSource) const;
  virtual GVT::Data::RayVector ao(const GVT::Data::ray& ray,
                                  const GVT::Math::Vector4f& sufaceNormal,
                                  float samples) const;
  virtual GVT::Data::RayVector secondary(
      const GVT::Data::ray& ray, const GVT::Math::Vector4f& sufaceNormal,
      float samples) const;

  GVT::Math::Vector4f CosWeightedRandomHemisphereDirection2(
      const GVT::Math::Vector4f& n) const {
    float Xi1 = (float)rand() / (float)RAND_MAX;
    float Xi2 = (float)rand() / (float)RAND_MAX;

    float theta = acos(sqrt(1.0 - Xi1));
    float phi = 2.0 * 3.1415926535897932384626433832795 * Xi2;

    float xs = sinf(theta) * cosf(phi);
    float ys = cosf(theta);
    float zs = sinf(theta) * sinf(phi);

    GVT::Math::Vector3f y(n);
    GVT::Math::Vector3f h = y;
    if (fabs(h.x) <= fabs(h.y) && fabs(h.x) <= fabs(h.z))
      h[0] = 1.0;
    else if (fabs(h.y) <= fabs(h.x) && fabs(h.y) <= fabs(h.z))
      h[1] = 1.0;
    else
      h[2] = 1.0;

    GVT::Math::Vector3f x = (h ^ y);
    GVT::Math::Vector3f z = (x ^ y);

    GVT::Math::Vector4f direction = x * xs + y * ys + z * zs;
    direction.normalize();
    std::cout << "dot(n, direction) = " << direction* n;
    return direction;
  }

 protected:
};

class Lambert : public Material {
 public:
  Lambert(const GVT::Math::Vector4f& kd = GVT::Math::Vector4f());
  Lambert(const Lambert& orig);
  virtual ~Lambert();

  virtual GVT::Math::Vector4f shade(
      const GVT::Data::ray& ray, const GVT::Math::Vector4f& sufaceNormal,
      const GVT::Data::LightSource* lightSource) const;
  virtual GVT::Data::RayVector ao(const GVT::Data::ray& ray,
                                  const GVT::Math::Vector4f& sufaceNormal,
                                  float samples) const;
  virtual GVT::Data::RayVector secondary(
      const GVT::Data::ray& ray, const GVT::Math::Vector4f& sufaceNormal,
      float samples) const;

 protected:
  GVT::Math::Vector4f kd;
};

class Phong : public Material {
 public:
  Phong(const GVT::Math::Vector4f& kd = GVT::Math::Vector4f(),
        const GVT::Math::Vector4f& ks = GVT::Math::Vector4f(),
        const float& alpha = 1.f);
  Phong(const Phong& orig);
  virtual ~Phong();

  virtual GVT::Math::Vector4f shade(
      const GVT::Data::ray& ray, const GVT::Math::Vector4f& sufaceNormal,
      const GVT::Data::LightSource* lightSource) const;
  virtual GVT::Data::RayVector ao(const GVT::Data::ray& ray,
                                  const GVT::Math::Vector4f& sufaceNormal,
                                  float samples) const;
  virtual GVT::Data::RayVector secondary(
      const GVT::Data::ray& ray, const GVT::Math::Vector4f& sufaceNormal,
      float samples) const;

 protected:
  GVT::Math::Vector4f kd;
  GVT::Math::Vector4f ks;
  float alpha;
};

class BlinnPhong : public Material {
 public:
  BlinnPhong(const GVT::Math::Vector4f& kd = GVT::Math::Vector4f(),
             const GVT::Math::Vector4f& ks = GVT::Math::Vector4f(),
             const float& alpha = 1.f);
  BlinnPhong(const BlinnPhong& orig);
  virtual ~BlinnPhong();

  virtual GVT::Math::Vector4f shade(
      const GVT::Data::ray& ray, const GVT::Math::Vector4f& sufaceNormal,
      const GVT::Data::LightSource* lightSource) const;
  virtual GVT::Data::RayVector ao(const GVT::Data::ray& ray,
                                  const GVT::Math::Vector4f& sufaceNormal,
                                  float samples) const;
  virtual GVT::Data::RayVector secondary(
      const GVT::Data::ray& ray, const GVT::Math::Vector4f& sufaceNormal,
      float samples) const;

 protected:
  GVT::Math::Vector4f kd;
  GVT::Math::Vector4f ks;
  float alpha;
} class WavefrontObjMaterial : public Material {
 public:
  WavefrontObjMaterial();

  WavefrontObjMaterial(const WavefrontObjMaterial& orig);

  virtual ~WavefrontObjMaterial();

  virtual GVT::Math::Vector4f shade(
      const GVT::Data::ray& ray, const GVT::Math::Vector4f& sufaceNormal,
      const GVT::Data::LightSource* lightSource) const;

  virtual GVT::Data::RayVector ao(const GVT::Data::ray& ray,
                                  const GVT::Math::Vector4f& sufaceNormal,
                                  float samples) const;

  virtual GVT::Data::RayVector secondary(
      const GVT::Data::ray& ray, const GVT::Math::Vector4f& sufaceNormal,
      float samples) const;

  void set_kd(const GVT::Math::Vector4f& value) const { kd = value; }

  void set_ks(const GVT::Math::Vector4f& value) const { ks = value; }

  void set_ke(const GVT::Math::Vector4f& value) const { ke = value; }

  void set_ka(const GVT::Math::Vector4f& value) const { ka = value; }

  void set_specular_exponent(float value) const { specular_exponent = value; }

  void set_optical_density(float value) const { optical_density = value; }

  void set_alpha(float value) const { alpha = value; }

  void set_has_illum_model(float value) const { has_illum_model = value; }

  void set_has_ambient_texture_map(bool value) const {
    has_ambient_texture_map = value;
  }

  void set_ambient_texture_map(const std::string& value) const {
    has_ambient_texture_map = value;
  }

  void set_has_diffuse_texture_map(bool value) const {
    has_diffuse_texture_map = value;
  }

  void set_diffuse_texture_map(const std::string& value) const {
    has_diffuse_texture_map = value;
  }

 protected:
  GVT::Math::Vector4f kd;
  GVT::Math::Vector4f ks;
  GVT::Math::Vector4f ke;
  GVT::Math::Vector4f ka;
  float specular_exponent;  // Blinn-Phong shineness exponent
  float optical_density;    // index of refraction
  float alpha;              // opacity
  // The values below are currently ignored.
  bool has_illum_model;
  int illum_model;
  bool has_ambient_texture_map;
  std::string ambient_texture_map;
  bool has_diffuse_texture_map;
  std::string diffuse_texture_map;
};

}  // namespace Data

}  // namespace GVT

#endif /* GVT_MATERIAL_H */

