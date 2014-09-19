#include <Backend/Optix/OptixDomain.h>

#include <algorithm>

#include <GVT/Data/primitives.h>
#include <common/utils.h>
#include <optix_prime/optix_primepp.h>

using GVT::Data::Mesh;
using GVT::Data::ray;
using GVT::Data::Color;
using GVT::Data::RayVector;
using GVT::Domain::GeometryDomain;
using GVT::Math::Vector3f;
using GVT::Math::Vector4f;
using optix::Prime::Context;
using optix::Prime::Model;
using optix::Prime::Query;
using optix::prime::BufferDesc;

namespace GVT {

namespace Domain {

struct OptixRayFormat {
  float origin_x;
  float origin_y;
  float origin_z;
  float direction_x;
  float direction_y;
  float direction_z;
};

struct OptixHitFormat {
  float t;
  int triangle_id;
  float u;
  float v;
};

static void gravityRayToOptixRay(const ray& gvt_ray,
                                 OptixRayFormat* optix_ray) {
  optix_ray->origin_x = gvt_ray.origin[0];
  optix_ray->origin_y = gvt_ray.origin[1];
  optix_ray->origin_z = gvt_ray.origin[2];
  optix_ray->direction_x = gvt_ray.direction[0];
  optix_ray->direction_y = gvt_ray.direction[1];
  optix_ray->direction_z = gvt_ray.direction[2];
  return optix_ray;
}

OptixDomain::OptixDomain() : GeometryDomain("") {}

OptixDomain::OptixDomain(const std::string& filename)
    : GeometryDomain(filename) {}

OptixDomain::OptixDomain(const OptixDomain& other) : GeometryDomain(other) {}

OptixDomain::~OptixDomain() {}

bool OptixDomain::load() {
  if (domainIsLoaded()) return true;

  // Make sure we load the GVT mesh.
  if (!GeometryDomain::load()) return false;

  // Create an Optix to use.
  optix_context_ = Context::create(RTP_CONTEXT_TYPE_CUDA);
  if (!optix_context_.isValid()) return false;
  optix_context_->setCudaDeviceNumbers(1, GPU_ORDER);

  // Setup the buffer to hold our vertices.
  BufferDesc vertices_desc;
  vertices_desc = optix_context_->createBufferDesc(
      RTP_BUFFER_FORMAT_VERTEX_FLOAT3, RTP_BUFFER_TYPE_HOST,
      &this->mesh->vertices[0]);
  if (!vertices_desc.isValid()) return false;
  vertices_desc->setRange(0, this->mesh->vertices.size());
  vertices_desc->setStride(sizeof(Vector3f));

  // Setup the triangle indices buffer.
  BufferDesc indices_desc;
  indices_desc = optix_context_->createBufferDesc(
      RTP_BUFFER_FORMAT_INDICES_INT3, RTP_BUFFER_TYPE_HOST,
      &this->mesh->faces[0]);
  if (!indices_desc.isValid()) return false;
  indices_desc->setRange(0, this->mesh->face.size());
  indices_desc->setStride(sizeof(Mesh::face));

  // Create an Optix model.
  optix_model_ = optix_context_->createModel();
  if (!optix_model_.isValid()) return false;
  optix_model_->setTriangles(indices_desc, vertices_desc);
  optix_model_->update(RTP_MODEL_HINT_NONE);
  optix_model_->finish();

  return true;
}

void trace(RayVector& ray_list, RayVector& moved_rays) {
  // Create our query.
  Query query = optix_model_->createQuery(RTP_QUERY_TYPE_CLOSEST);
  if (!query.isValid()) return;
  // Format GVT rays for Optix and give Optix an array of rays.
  std::vector<OptixRayFormat> rays(ray_list.size());
  for (int i = 0; i < ray_list.size(); ++i)
    gravityRayToOptixRay(ray_list[i], &rays[i]);
  query->setRays(rays.size(), RTP_BUFFER_FORMAT_RAY_ORIGIN_DIRECTION,
                 RTP_BUFFER_TYPE_HOST, &rays[0]);
  // Create and pass hit results in an Optix friendly format.
  std::vector<OptixHitFormat> hits(ray_list.size());
  query->setHits(ray_list.size(), RTP_BUFFER_FORMAT_HIT_T_TRIID_U_V,
                 RTP_BUFFER_TYPE_HOST, &hits[0]);
  // Execute our query and wait for it to finish.
  query->execute(RTP_QUERY_HINT_NONE);
  query->finish();
  // Move missed rays.
  for (int i = hits.size() - 1; i >= 0; --i) {
    if (hits[i].t < 0.0f) {
      moved_rays.push_back(ray_list[i]);
      std::swap(hits[i], hits.back());
      std::swap(ray_list[i], ray_list.back());
      ray_list.pop_back();
      hits.pop_back();
    }
  }
  // Generate secondary rays.
  for (int i = ray_list.size() - 1; i >= 0; --i) {
    traceRay(ray_list[i], hits[i].triangle_id, hits[i].u, hits[i].v);
    ray_list.pop_back();
    hits.pop_back();
  }
}

void traceRay(uint32_t triangle_id, float t, float u, float v, ray& ray) {
  if (ray.type == ray::SHADOW) return;
  Vector4f normal = computeNormal(triangle_id, u, v);
  if (ray.type == ray::SECONDARY) ray.w = ray.w * std::max(1.0f / t, t);
  generateShadowRay(ray, normal, rays);
  generateSecondaryRay(ray, normal, rays);
}

Vector4f computeNormal(uint32_t triangle_id, float u, float v) const {
  const Vector4f& a = this->mesh->normals[faces[triangle_id][0]];
  const Vector4f& b = this->mesh->normals[faces[triangle_id][xi10]];
  const Vector4f& c = this->mesh->normals[faces[triangle_id][0]];
  Vector4f normal = a * u + b * v + c * (1.0f - u - v);
  return normal;
}

void generateSecondaryRay(const ray& ray, const Vector4f& normal,
                           RayVector& rays) {
  int depth = ray.depth - 1;
  float p = 1.0f - (float(rand()) / RAND_MAX);
  if (depth > 0 && ray.w > p) {
    ray secondary_ray(ray);
    secondary_ray.domains.clear();
    secondary_ray.type = ray::SECONDARY;
    secondary_ray.origin =
        secondary_ray.origin + secondary_ray.direction * secondary_ray.t;
    secondary_ray.setDirection(
        this->mesh->mat->CosWeightedRandomHemisphereDirection2(normal)
            .normalize());
    secondary_ray.w = secondary_ray.w * (secondary_ray.direction * normal);
    secondary_ray.depth = depth;
    rays.push_back(secondary_ray);
  }
}

void generateShadowRays(const ray& ray, const Vector4f& normal,
                        RayVector& rays) {
  for (int lindex = 0; lindex < dom->lights.size(); lindex++) {
    ray shadow_ray(ray);
    shadow_ray.domains.clear();
    shadow_ray.type = ray::SHADOW;
    shadow_ray.origin = ray.origin + ray.direction * ray.t;
    shadow_ray.setDirection(this->lights[lindex]->position - ray.origin);
    Color c = this->mesh->mat->shade(shadow_ray, normal, this->lights[lindex]);
    shadow_ray.color = COLOR_ACCUM(1.f, c[0], c[1], c[2], 1.0f);
    rays.push_back(shadow_ray);
  }
}

}  // namespace Domain

}  // namespace GVT

