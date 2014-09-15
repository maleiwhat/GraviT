#include <Backend/Optix/OptixDomain.h>

#include <algorithm>

#include <Backend/Optix/gvt_optix.h>
#include <GVT/Data/primitives.h>
#include <common/utils.h>
#include <optix_prime/optix_primepp.h>

using GTV::Data::Mesh;
using GTV::Data::ray;
using GVT::Data::RayVector;
using GVT::Domain::GeometryDomain;
using GVT::Math::Vector3f;
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

static void GravityRayToOptixRay(const ray& gvt_ray,
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
  GeometryDomain::load();

  // Create an Optix to use. 
  optix_context_ = Context::create(RTP_CONTEXT_TYPE_CUDA);
  optix_context_->setCudaDeviceNumbers(1, GPU_ORDER);

  // Setup the buffer to hold our vertices.
  BufferDesc vertices_desc;
  vertices_desc = optix_context_->createBufferDesc(
      RTP_BUFFER_FORMAT_VERTEX_FLOAT3, RTP_BUFFER_TYPE_HOST,
      &this->mesh->vertices[0]);
  vertices_desc->setRange(0, this->mesh->vertices.size());
  vertices_desc->setStride(sizeof(Vector3f));

  // Setup the triangle indices buffer.
  BufferDesc indices_desc;
  indices_desc = optix_context_->createBufferDesc(
      RTP_BUFFER_FORMAT_INDICES_INT3, RTP_BUFFER_TYPE_HOST,
      &this->mesh->faces[0]);
  indices_desc->setRange(0, this->mesh->face.size());
  indices_desc->setStride(sizeof(Mesh::face));

  // Create an Optix model.
  optix_model_ = optix_context_->createModel();
  optix_model_->setTriangles(indices_desc, vertices_desc);
  optix_model_->update(RTP_MODEL_HINT_NONE);
  optix_model_->finish();

  // TODO(rsmith): Handle error conditions, but return true for now.
  return true;
}

void trace(RayVector& ray_list, RayVector& moved_rays) {
  // Create our query.
  Query query = optix_model_->createQuery(RTP_QUERY_TYPE_CLOSEST);
  // Format GVT rays for Optix and give Optix an array of rays.
  std::vector<OptixRayFormat> rays(ray_list.size());
  for (int i = 0; i < ray_list.size(); ++i)
    GravityRayToOptixRay(ray_list[i], &rays[i]);
  query->setRays(ray_list.size(), RTP_BUFFER_FORMAT_RAY_ORIGIN_DIRECTION,
                 RTP_BUFFER_TYPE_HOST, &rays[0]);
  // Create and pass hit results in an Optix friendly format.
  std::vector<OptixHitFormat> hits(ray_list.size());
  query->setHits(ray_list.size(), RTP_BUFFER_FORMAT_HIT_T_TRIID_U_V,
                 RTP_BUFFER_TYPE_HOST, &hits[0]);
  // Execute our query and wait for it to finish.
  query->execute(RTP_QUERY_HINT_NONE);
  query->finish();
  // Move missed rays.
  for (int i = 0; i < hits.size(); ++i) {
    if (hits[i].t < 0.0f) {
      moved_rays.push_back(ray_list[i]);
      std::swap(ray_list[i], ray_list.back());
      ray_list.pop_back();
    } 
  }
  // TODO(rsmith): Shade hit rays and fire shadow rays as needed.
}

}  // namespce Domain

}  // namespace GVT

