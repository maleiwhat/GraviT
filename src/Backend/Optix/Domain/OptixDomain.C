#include <string>
#include <boost/timer/timer.hpp>


#include <Backend/Optix/Domain/OptixDomain.h>
#include <GVT/Data/primitives.h>
#include <algorithm>
#include <common/utils.h>
#include <optix_prime/optix_primepp.h>

using GVT::Data::Color;
using GVT::Data::LightSource;
using GVT::Data::Mesh;
using GVT::Data::RayVector;
using GVT::Data::ray;
using GVT::Domain::GeometryDomain;
using GVT::Math::Vector3f;
using GVT::Math::Vector4f;
using optix::prime::BufferDesc;
using optix::prime::Context;
using optix::prime::Model;
using optix::prime::Query;

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
}

OptixDomain::OptixDomain() : GeometryDomain("") {}

OptixDomain::OptixDomain(const std::string& filename)
    : GeometryDomain(filename), loaded_(false) {}

OptixDomain::OptixDomain(const OptixDomain& domain)
    : GeometryDomain(domain),
      optix_context_(domain.optix_context_),
      optix_model_(domain.optix_model_),
      loaded_(false) {}

OptixDomain::~OptixDomain() {}

OptixDomain::OptixDomain(const std::string& filename,
                         GVT::Math::AffineTransformMatrix<float> m)
    : GVT::Domain::GeometryDomain(filename, m), loaded_(false) {
  this->load();
}

bool OptixDomain::load() {

  if (loaded_) return true;

  // Make sure we load the GVT mesh.
  GVT_ASSERT(GeometryDomain::load(), "Geometry not loaded");
  if (!GeometryDomain::load()) return false;

  // Make sure normals exist.
  if (this->mesh->normals.size() < this->mesh->vertices.size() ||
      this->mesh->face_normals.size() < this->mesh->faces.size())
    this->mesh->generateNormals();

  // Create an Optix context to use.
  optix_context_ = Context::create(RTP_CONTEXT_TYPE_CUDA);

  GVT_ASSERT(optix_context_.isValid(), "Optix Context is not valid");
  if (!optix_context_.isValid()) return false;

  // Setup the buffer to hold our vertices.
  BufferDesc vertices_desc;
  vertices_desc = optix_context_->createBufferDesc(
      RTP_BUFFER_FORMAT_VERTEX_FLOAT3, RTP_BUFFER_TYPE_HOST,
      &this->mesh->vertices[0]);

  GVT_ASSERT(vertices_desc.isValid(), "Vertices are not valid");
  if (!vertices_desc.isValid()) return false;

  vertices_desc->setRange(0, this->mesh->vertices.size());
  vertices_desc->setStride(sizeof(Vector3f));

  // Setup the triangle indices buffer.
  BufferDesc indices_desc;
  indices_desc = optix_context_->createBufferDesc(
      RTP_BUFFER_FORMAT_INDICES_INT3, RTP_BUFFER_TYPE_HOST,
      &this->mesh->faces[0]);

  GVT_ASSERT(indices_desc.isValid(), "Indices are not valid");
  if (!indices_desc.isValid()) return false;

  indices_desc->setRange(0, this->mesh->faces.size());
  indices_desc->setStride(sizeof(Mesh::face));

  // Create an Optix model.
  optix_model_ = optix_context_->createModel();

  GVT_ASSERT(optix_model_.isValid(), "Model is not valid");
  if (!optix_model_.isValid()) return false;

  optix_model_->setTriangles(indices_desc, vertices_desc);
  optix_model_->update(RTP_MODEL_HINT_NONE);
  optix_model_->finish();

  if (!optix_model_.isValid()) return false;

  loaded_ = true;
  return true;
}

void OptixDomain::trace(RayVector& ray_list, RayVector& moved_rays) {
  // Create our query.
  boost::timer::auto_cpu_timer optix_time("Optix domain trace: %w\n");
    
  try {
    this->load();
    if (!this->mesh->haveNormals || this->mesh->normals.size() == 0)
      this->mesh->generateNormals();

    GVT_ASSERT(optix_model_.isValid(), "trace:Model is not valid");
    if (!optix_model_.isValid()) return;

    RayVector next_list;
    while (!ray_list.empty()) {
      next_list.push_back(ray_list.back());
      ray_list.pop_back();
    }
    RayVector chunk;
    const uint32_t kMaxChunkSize = 65536;
    while (!next_list.empty()) {
      ray_list.swap(next_list);
      while (!ray_list.empty()) {
        chunk.push_back(ray_list.back());
        ray_list.pop_back();
        if (chunk.size() == kMaxChunkSize || ray_list.empty()) {
          traceChunk(chunk, next_list, moved_rays);
          chunk.clear();
        }
      }
    }
  }
  catch (const optix::prime::Exception& e) {
    GVT_ASSERT(false, e.getErrorString());
  }
}

void OptixDomain::traceChunk(RayVector& chunk, RayVector& next_list,
                             RayVector& moved_rays) {
  // Create our query.
  Query query = optix_model_->createQuery(RTP_QUERY_TYPE_CLOSEST);
  if (!query.isValid()) return;

  // Format GVT rays for Optix and give Optix an array of rays.
  std::vector<OptixRayFormat> optix_rays(chunk.size());
  for (int i = 0; i < chunk.size(); ++i)
    gravityRayToOptixRay(this->toLocal(chunk[i]), &optix_rays[i]);

  // Hand the rays to Optix.
  query->setRays(optix_rays.size(), RTP_BUFFER_FORMAT_RAY_ORIGIN_DIRECTION,
                 RTP_BUFFER_TYPE_HOST, &optix_rays[0]);

  // Create and pass hit results in an Optix friendly format.
  std::vector<OptixHitFormat> hits(chunk.size());
  query->setHits(hits.size(), RTP_BUFFER_FORMAT_HIT_T_TRIID_U_V,
                 RTP_BUFFER_TYPE_HOST, &hits[0]);

  // Execute our query and wait for it to finish.
  query->execute(RTP_QUERY_HINT_NONE);
  query->finish();

  // Move missed rays.
  for (int i = hits.size() - 1; i >= 0; --i) {
    if (hits[i].t < ray::RAY_EPSILON) {
      moved_rays.push_back(chunk[i]);
      std::swap(hits[i], hits.back());
      std::swap(chunk[i], chunk.back());
      chunk.pop_back();
      hits.pop_back();
    }
  }

  // Trace each ray: shade, fire shadow rays, fire secondary rays.
  for (int i = 0; i < chunk.size(); ++i)
    this->traceRay(hits[i].triangle_id, hits[i].t, hits[i].u, hits[i].v,
                   chunk[i], next_list);
}

void OptixDomain::traceRay(uint32_t triangle_id, float t, float u, float v,
                           ray& ray, RayVector& rays) {
  if (ray.type == ray::SHADOW) return;
  ray.t = t;
  Vector4f normal = computeNormal(triangle_id, u, v);
  //if (normal.length() < ray::RAY_EPSILON) return;
  normal.normalize();
  if (ray.type == ray::SECONDARY) ray.w = ray.w * std::max(1.0f / t, t);
  generateShadowRays(ray, normal, rays);
  generateSecondaryRays(ray, normal, rays);
}

Vector4f OptixDomain::computeNormal(uint32_t triangle_id, float u,
                                    float v) const {
  float length = this->mesh->face_normals[triangle_id].length();
  if (fabs(length - 1.0f) > 1e-5) {
    std::cerr << "length = " << length << "\n";
    GVT_ASSERT(false, "Normal not normal");
  }
//  return this->mesh->face_normals[triangle_id];
  const Mesh::face_to_normals& normals =
      this->mesh->faces_to_normals[triangle_id];
  const Vector4f& a = this->mesh->normals[normals.get<0>()];
  const Vector4f& b = this->mesh->normals[normals.get<1>()];
  const Vector4f& c = this->mesh->normals[normals.get<2>()];
  Vector4f normal = a * u + b * v + c * (1.0f - u - v);
  //std::cout << "computeNormal:normal = (" << normal.n[0] << "," << normal.n[1]
  //          << "," << normal.n[2] << ")\n";
  normal.normalize();
  return normal;
}

void OptixDomain::generateSecondaryRays(const ray& ray_in,
                                        const Vector4f& normal,
                                        RayVector& rays) {
  int depth = ray_in.depth - 1;
  float p = 1.0f - (float(rand()) / RAND_MAX);
  if (depth > 0 && ray_in.w > p) {
    ray secondary_ray(ray_in);
    secondary_ray.domains.clear();
    secondary_ray.type = ray::SECONDARY;
    float t_secondary = secondary_ray.t - ray::RAY_EPSILON;
    secondary_ray.origin =
        secondary_ray.origin + secondary_ray.direction * t_secondary;
    secondary_ray.setDirection(
        this->mesh->mat->CosWeightedRandomHemisphereDirection2(normal)
            .normalize());
    secondary_ray.w = secondary_ray.w * (secondary_ray.direction * normal);
    secondary_ray.depth = depth;
    rays.push_back(secondary_ray);
  }
}

void OptixDomain::generateShadowRays(const ray& ray_in, const Vector4f& normal,
                                     RayVector& rays) {
  for (int lindex = 0; lindex < this->lights.size(); ++lindex) {
    ray shadow_ray(ray_in);
    shadow_ray.domains.clear();
    shadow_ray.type = ray::SHADOW;
    float t_shadow = shadow_ray.t - ray::RAY_EPSILON;
    shadow_ray.origin = shadow_ray.origin + shadow_ray.direction * t_shadow;
    Vector4f light_position(this->lights[lindex]->position);
    shadow_ray.setDirection(light_position - shadow_ray.origin);
    Color c = this->mesh->mat->shade(shadow_ray, normal, this->lights[lindex]);
    float w = shadow_ray.w;
    // Need to weight this somehow.
    shadow_ray.color =
        COLOR_ACCUM(1.0f, normal.n[0], normal.n[1], normal.n[2], 1.0f);
    rays.push_back(shadow_ray);
  }
}

}  // namespace Domain

}  // namespace GVT
