/**
  @file main.cpp
*/

#include <atomic>
#include <iomanip>
#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <thread>
#include <vector>

#include "glm/gtx/transform.hpp"
#include "glm/glm.hpp"
#include "Image.h"
#include "Material.h"

using namespace std;

#define TOLERANCE 1e-4f
#define MAX_RECURSION_DEPTH 5
#define AA_SAMPLES 64
#define APERTURE_RADIUS 0.2f
#define FOCAL_DISTANCE 13.0f

/**
  Class representing a single ray.
*/
class Ray
{
public:
  glm::vec3 origin;
  glm::vec3 direction;
  Ray(glm::vec3 origin, glm::vec3 direction)
      : origin(origin), direction(direction) {}
};

class Object;
glm::vec3 toneMapping(glm::vec3);
glm::vec3 trace_ray(const Ray &ray, int depth);

struct Hit
{
  bool hit;
  glm::vec3 normal;
  glm::vec3 intersection;
  float distance;
  Object *object;
  bool isInsideObject;
};

/**
  General class for the object
*/
class Object
{
protected:
  glm::mat4 transformationMatrix;
  glm::mat4 inverseTransformationMatrix;
  glm::mat4 normalMatrix;

public:
  glm::vec3 color;   ///< Color of the object
  Material material; ///< Structure describing the material of the object
  /** A function computing an intersection, which returns the structure Hit */
  virtual Hit intersect(Ray ray) = 0;
  virtual ~Object() = default;

  /** Function that returns the material struct of the object*/
  Material getMaterial() { return material; }
  /** Function that set the material
          @param material A structure describing the material of the object
          */
  void setMaterial(Material material) { this->material = material; }

  void setTransformation(glm::mat4 matrix)
  {
    transformationMatrix = matrix;
    inverseTransformationMatrix = glm::inverse(matrix);
    normalMatrix = glm::transpose(inverseTransformationMatrix);
  }
};

class Box : public Object
{
private:
  glm::vec3 minBound;
  glm::vec3 maxBound;

public:
  // Create a box defined by min and max points (local space)
  Box(glm::vec3 min, glm::vec3 max, Material mat) : minBound(min), maxBound(max)
  {
    this->material = mat;
  }

  Hit intersect(Ray ray)
  {
    Hit hit;
    hit.hit = false;

    // 1. Transform Ray from World Space to Object (Local) Space
    // We do this so we can intersect with a simple AABB at (0,0,0)
    glm::vec3 localOrigin = glm::vec3(inverseTransformationMatrix * glm::vec4(ray.origin, 1.0f));
    glm::vec3 localDir = glm::vec3(inverseTransformationMatrix * glm::vec4(ray.direction, 0.0f));

    // Normalize direction for accurate slab calculation
    // (Note: This means 't' will be in local distance units, so we must recalculate world distance later)
    localDir = glm::normalize(localDir);

    // 2. Slab Method Intersection
    float tMin = 0.001f;
    float tMax = 100000.0f;

    glm::vec3 bounds[2] = {minBound, maxBound};
    glm::vec3 axisNormal(0.0f); // Temporary normal storage
    int hitAxis = -1;

    for (int i = 0; i < 3; ++i)
    { // Check X, Y, Z slabs
      float invD = 1.0f / localDir[i];
      float t0 = (bounds[0][i] - localOrigin[i]) * invD;
      float t1 = (bounds[1][i] - localOrigin[i]) * invD;

      float sign = -1.0f; // Default: we hit the 'min' plane (normal points negative)

      if (invD < 0.0f)
      {
        std::swap(t0, t1);
        sign = 1.0f; // We hit the 'max' plane (normal points positive)
      }

      if (t0 > tMin)
      {
        tMin = t0;
        hitAxis = i;
        axisNormal = glm::vec3(0.0f);
        axisNormal[i] = sign;
      }
      tMax = std::min(tMax, t1);

      if (tMax <= tMin)
        return hit; // Ray missed the box
    }

    // 3. Valid Hit
    if (hitAxis != -1)
    {
      hit.hit = true;

      // Calculate hit point in LOCAL space
      glm::vec3 localHitPoint = localOrigin + tMin * localDir;

      // Transform hit point back to WORLD space
      hit.intersection = glm::vec3(transformationMatrix * glm::vec4(localHitPoint, 1.0f));

      // Transform normal back to WORLD space
      hit.normal = glm::normalize(glm::vec3(normalMatrix * glm::vec4(axisNormal, 0.0f)));

      // Recalculate accurate world distance
      hit.distance = glm::distance(ray.origin, hit.intersection);
      hit.object = this;
    }

    return hit;
  }
};

// Helper for random float in [0, 1]
float random_float()
{
  return static_cast<float>(rand()) / static_cast<float>(RAND_MAX);
}

// Helper for random float in a specific range [min, max]
float random_range(float min, float max)
{
  return min + (max - min) * random_float();
}

// Helper to get a random point in a disk centered at (0, 0) with radius R
glm::vec3 random_in_unit_disk(float radius)
{
  float r = random_float() * radius;
  float theta = random_float() * 2.0f * M_PI;
  return glm::vec3(r * cos(theta), r * sin(theta), 0.0f);
}

class Sphere : public Object
{
private:
  float radius;     ///< Radius of the sphere
  glm::vec3 center; ///< Center of the sphere

public:
  /**
          The constructor of the sphere
          @param radius Radius of the sphere
          @param center Center of the sphere
          @param color Color of the sphere
          */
  Sphere(float radius, glm::vec3 center, glm::vec3 color)
      : radius(radius), center(center)
  {
    this->color = color;
  }
  Sphere(float radius, glm::vec3 center, Material material)
      : radius(radius), center(center)
  {
    this->material = material;
  }
  /** Implementation of the intersection function*/
  Hit intersect(Ray ray)
  {
    glm::vec3 c = center - ray.origin;

    float cdotc = glm::dot(c, c);
    float cdotd = glm::dot(c, ray.direction);

    Hit hit;

    float D = 0;
    if (cdotc > cdotd * cdotd)
    {
      D = sqrt(cdotc - cdotd * cdotd);
    }
    if (D <= radius)
    {
      hit.hit = true;
      float t1 = cdotd - sqrt(radius * radius - D * D);
      float t2 = cdotd + sqrt(radius * radius - D * D);

      float t = t1;
      if (t < 0)
        t = t2;
      if (t < 0)
      {
        hit.hit = false;
        return hit;
      }

      hit.intersection = ray.origin + t * ray.direction;
      hit.normal = glm::normalize(hit.intersection - center);
      hit.distance = glm::distance(ray.origin, hit.intersection);
      hit.object = this;
      float dist_to_center = glm::length(ray.origin - center);
      hit.isInsideObject = (dist_to_center < radius);

      if (hit.isInsideObject)
      {
        hit.normal = -hit.normal;
      }
    }
    else
    {
      hit.hit = false;
    }
    return hit;
  }
};

class Plane : public Object
{
private:
  glm::vec3 normal;
  glm::vec3 point;

public:
  Plane(glm::vec3 point, glm::vec3 normal) : point(point), normal(normal) {}
  Plane(glm::vec3 point, glm::vec3 normal, Material material)
      : point(point), normal(normal)
  {
    this->material = material;
  }
  void setNormal(glm::vec3 n) { normal = n; }
  Hit intersect(Ray ray)
  {
    Hit hit;
    hit.hit = false;

    float denominator = glm::dot(normal, ray.direction);
    if (abs(denominator) >= TOLERANCE)
    {
      /*
       * R(t) = origin + t * direction
       * (R(t) - Plane.point) * normal = 0 -- if R(t) intersects the plane it
       * has to be orthogonal to the normal vector Expanding the equation
       * (ray.origin + t * ray.direction - plane.point) * normal = 0
       * (t * ray.direction) * normal = -(ray.origin - plane.point) * normal
       * (t * ray.direction) * normal = (plane.point - ray.origin) * normal
       * t * (ray.direction * normal) = (plane.point - ray.origin) * normal
       * t = ((plane.point - ray.origin) * normal) / (ray.direction * normal)
       */
      float t = glm::dot(this->point - ray.origin, normal) / denominator;
      if (t < 0)
        return hit;
      hit.intersection = ray.origin + t * ray.direction;
      // hit.distance = glm::distance(ray.origin, hit.intersection);
      hit.distance = t;
      hit.object = this;
      hit.normal = this->normal;
      hit.hit = true;
    }

    return hit;
  }
};

class Cone : public Object
{
private:
  Plane *plane;

  Hit coneSurfaceIntersect(Ray localRay, glm::vec3 worldOrigin)
  {
    Hit hit;
    hit.hit = false;

    glm::vec3 localOrigin = localRay.origin;
    glm::vec3 localDirection = localRay.direction;

    // x^2 + z^2 - y^2 = 0 - implicit cone equation
    // plug in the ray equation:
    // (Ox + tDx)^2 + (Oz + tDz)^2 - (Oy + tDy)^2 = 0
    // Expand the equation:
    // (Ox^2 + 2OxtDx + t^2Dx^2)+
    // (Oz^2 + 2OztDz + t^2Dz^2)-(Oy^2 + 2OytDy + t^2Dy^2) = 0
    // Group into At^2 + Bt + C = 0
    // A = dx - dy + dz
    // B = 2(ox*dx - oy*dy + oz*dz)
    // C = ox^2 - oy^2 + oz^2
    // Discriminant = B^2 - 4AC
    // t = (-B +- sqrt(Discriminant)) / 2A
    float dx = localDirection.x;
    float dy = localDirection.y;
    float dz = localDirection.z;
    float A = dx * dx - dy * dy + dz * dz;

    float ox = localOrigin.x;
    float oy = localOrigin.y;
    float oz = localOrigin.z;
    float C = ox * ox - oy * oy + oz * oz;
    float B = 2.0f * (dx * ox - dy * oy + dz * oz);

    float D = B * B - 4.0f * A * C;
    if (D < 0)
      return hit;
    float t1 = (-B - glm::sqrt(D)) / (2.0f * A);
    float t2 = (-B + glm::sqrt(D)) / (2.0f * A);
    float t = -1.0f;

    glm::vec3 localIntersection;
    if (t1 > TOLERANCE)
    {
      localIntersection = localOrigin + t1 * localDirection;
      if (localIntersection.y >= -1.0f && localIntersection.y <= 0.0f)
      {
        t = t1;
      }
    }

    if (t2 > TOLERANCE)
    {
      if (t < 0 || t2 < t)
      {
        localIntersection = localOrigin + t2 * localDirection;
        if (localIntersection.y >= -1.0f && localIntersection.y <= 0.0f)
        {
          t = t2;
        }
      }
    }

    if (t < 0)
      return hit;

    glm::vec3 localNormal = glm::normalize(glm::vec3(localIntersection.x, -localIntersection.y, localIntersection.z));
    glm::vec3 worldIntersection = glm::vec3(transformationMatrix * glm::vec4(localIntersection, 1.0f));
    glm::vec3 worldNormal = glm::normalize(glm::vec3(normalMatrix * glm::vec4(localNormal, 0.0f)));

    hit.hit = true;
    hit.object = this;
    hit.intersection = worldIntersection;
    hit.normal = worldNormal;
    hit.distance = glm::distance(worldIntersection, worldOrigin);

    return hit;
  }

public:
  Cone(Material material)
  {
    this->material = material;
    // base plane for cone that is pointing down(x: 0, y: -1, z: 0)
    plane = new Plane(glm::vec3(0, -1, 0), glm::vec3(0, -1, 0), material);
  }
  ~Cone()
  {
    delete plane;
  }

  Hit intersect(Ray ray)
  {
    glm::vec3 localOrigin = glm::vec3(inverseTransformationMatrix * glm::vec4(ray.origin, 1.0f));
    glm::vec3 localDirection = glm::vec3(inverseTransformationMatrix * glm::vec4(ray.direction, 0.0f));
    localDirection = glm::normalize(localDirection);
    Ray localRay(localOrigin, localDirection);

    Hit surfaceHit = coneSurfaceIntersect(localRay, ray.origin);
    Hit baseHit = plane->intersect(localRay);
    if (baseHit.hit)
    {
      float t = baseHit.distance;
      glm::vec3 localBasePoint = baseHit.intersection;
      float radius2 = localBasePoint.x * localBasePoint.x +
                      localBasePoint.z * localBasePoint.z;
      // checking whether the point is within the circle on the plane
      if (radius2 < 1.0f)
      {
        baseHit.intersection = glm::vec3(transformationMatrix * glm::vec4(localBasePoint, 1.0f));
        baseHit.normal = glm::vec3(normalMatrix * glm::vec4(baseHit.normal, 0.0f));
        baseHit.normal = glm::normalize(baseHit.normal);
        baseHit.distance = glm::distance(baseHit.intersection, ray.origin);
      }
      else
      {
        baseHit.hit = false;
      }
    }
    if (surfaceHit.hit && baseHit.hit)
    {
      // return the closest hit
      if (surfaceHit.distance < baseHit.distance)
        return surfaceHit;
      return baseHit;
    }
    else if (surfaceHit.hit)
      return surfaceHit;
    return baseHit;
  }
};

/**
Light class
*/
class Light
{
public:
  glm::vec3 position;
  glm::vec3 color;
  glm::vec3 direction;
  float cutoffAngleCos;
  float exp;
  float radius;

  Light(glm::vec3 position) : position(position)
  {
    color = glm::vec3(1.0);
    direction = glm::vec3(0.0);
    cutoffAngleCos = -1.0f;
    exp = -1.0f;
    radius = 0.0f; // Default to point light
  }

  // Constructor for Area Light (Soft Shadows)
  Light(glm::vec3 position, glm::vec3 color, float r)
      : position(position), color(color), direction(0.0f), cutoffAngleCos(-1.0f), exp(-1.0f), radius(r) {}

  Light(glm::vec3 position, glm::vec3 color)
      : position(position), color(color), direction(0.0f), cutoffAngleCos(-1.0f), exp(-1.0f), radius(0.0f) {}

  Light(glm::vec3 position, glm::vec3 color, glm::vec3 dir, float angleDegrees, float exponent)
      : position(position), color(color), direction(glm::normalize(dir)),
        cutoffAngleCos(glm::cos(glm::radians(angleDegrees))), exp(exponent), radius(0.0f) {}
};

vector<Light *> lights; ///< A list of lights in the scene
glm::vec3 ambient_light(0.01f);
vector<Object *> objects; ///< A list of all objects in the scene

glm::vec3 WardModel(const Ray &ray, const Hit &hit)
{
  glm::vec3 color(0.0);
  Material mat = hit.object->getMaterial();

  // 1. Generate Anisotropic Basis
  glm::vec3 N = glm::normalize(hit.normal);
  glm::vec3 UP = glm::vec3(0, 1, 0);
  if (abs(glm::dot(N, UP)) > 0.99f)
    UP = glm::vec3(0, 0, 1);
  glm::vec3 T = glm::normalize(glm::cross(UP, N));
  glm::vec3 B = glm::normalize(glm::cross(N, T));

  for (int light_num = 0; light_num < lights.size(); light_num++)
  {
    Light *currentLight = lights[light_num];
    glm::vec3 L = glm::normalize(currentLight->position - hit.intersection);
    glm::vec3 V = glm::normalize(-ray.direction);
    glm::vec3 H = glm::normalize(L + V);

    // --- UPDATED SHADOW LOGIC (Matches PhongModel) ---
    float shadowPercent = 0.0f;

    if (currentLight->radius > 0.0f)
    {
      // Soft Shadows (Area Light)
      int shadow_samples = 16;
      float shadow_hits = 0.0f;
      for (int s = 0; s < shadow_samples; ++s)
      {
        glm::vec3 random_point = glm::vec3(random_range(-1, 1), random_range(-1, 1), random_range(-1, 1));
        random_point = glm::normalize(random_point) * currentLight->radius;
        glm::vec3 samplePos = currentLight->position + random_point;
        glm::vec3 sampleDir = glm::normalize(samplePos - hit.intersection);
        Ray shadowRay(hit.intersection + sampleDir * TOLERANCE, sampleDir);
        float distToLight = glm::distance(samplePos, hit.intersection);

        bool hit_something = false;
        for (int obj_num = 0; obj_num < objects.size(); ++obj_num)
        {
          Hit shadow_hit = objects[obj_num]->intersect(shadowRay);
          if (shadow_hit.hit && shadow_hit.distance < distToLight)
          {
            if (objects[obj_num]->getMaterial().krefract > 0.0f)
              continue;
            if (objects[obj_num]->getMaterial().type == EMISSIVE)
              continue;
            hit_something = true;
            break;
          }
        }
        if (hit_something)
          shadow_hits += 1.0f;
      }
      shadowPercent = shadow_hits / (float)shadow_samples;
    }
    else
    {
      // Hard Shadows
      Ray shadowRay(hit.intersection + L * TOLERANCE, L);
      float distToLight = glm::distance(currentLight->position, hit.intersection);
      for (int obj_num = 0; obj_num < objects.size(); ++obj_num)
      {
        Hit shadow_hit = objects[obj_num]->intersect(shadowRay);
        if (shadow_hit.hit && shadow_hit.distance < distToLight)
        {
          if (objects[obj_num]->getMaterial().krefract > 0.0f)
            continue;
          shadowPercent = 1.0f;
          break;
        }
      }
    }

    // Skip calculation if fully shadowed
    if (shadowPercent >= 1.0f)
      continue;

    // --- Ward Model Calculation ---
    float NdotL = std::max(0.0f, glm::dot(N, L));
    float NdotL_clamped = std::max(0.001f, glm::dot(N, L));
    float NdotV_clamped = std::max(0.001f, glm::dot(N, V));
    float NdotH = glm::dot(N, H);
    float HdotT = glm::dot(H, T);
    float HdotB = glm::dot(H, B);

    // Diffuse
    glm::vec3 diffuse = mat.diffuse * NdotL;

    // Specular (Ward)
    float denum = 4.0f * glm::pi<float>() * mat.alphaX * mat.alphaY * sqrt(NdotL_clamped * NdotV_clamped);
    float term1 = 1.0f / denum;
    float exponent = -((pow(HdotT / mat.alphaX, 2.0f) + pow(HdotB / mat.alphaY, 2.0f)) / pow(NdotH, 2.0f));
    float term2 = exp(exponent);
    float spec_val = term1 * term2;

    if (NdotL > 0.0f)
    {
      glm::vec3 specular = mat.specular * spec_val * NdotL;
      // Apply Shadow
      color += currentLight->color * (diffuse + specular) * (1.0f - shadowPercent);
    }
  }
  color += ambient_light * mat.ambient;
  return color;
}
// ===========================
// Participating Media (Fog)
// ===========================

struct FogBlob
{
  glm::vec3 center;
  float radius;  // Controls the spatial extent (standard deviation-ish)
  float density; // Peak density at the center
};

// A few Gaussian "smoke" blobs in the scene, placed towards the back so that
// fog mainly appears in the background rather than around the foreground objects.
vector<FogBlob> fogBlobs; 

struct FogSettings {
    bool enabled = true;
    glm::vec3 color = glm::vec3(0.8f, 0.85f, 0.9f); // Light blue-grey
    float absorption = 0.3f;      // Global thickness multiplier (sigma_a)
    int stepCount = 64;           // Quality vs Performance (16-128)
};

FogSettings globalFog; // Create a global instance

// Returns volumetric density at point p using a sum of Gaussian blobs.
// This implements a simple "metaball" style density field.
float getDensity(const glm::vec3 &p)
{
  float rho = 0.0f;
  for (const FogBlob &b : fogBlobs)
  {
    glm::vec3 d = p - b.center;
    float dist2 = glm::dot(d, d); // squared distance
    float sigma2 = b.radius * b.radius;
    // Standard Gaussian: exp(-r^2 / (2 * sigma^2))
    float contrib = b.density * expf(-dist2 / (2.0f * sigma2));
    rho += contrib;
  }
  return rho;
}

// Compute Beer-Lambert transmittance along the ray segment from
// ray.origin to ray.origin + ray.direction * maxDistance.
// Absorption only (no in-scattering): I = I0 * exp(-∫ σ * ρ(t) dt).
float computeTransmittance(const Ray &ray, float maxDistance) {
    if (!globalFog.enabled || maxDistance <= 0.0f || fogBlobs.empty()) {
        return 1.0f; // 100% transparent
    }

    // Param 1: Quality. Higher = smoother fog, Slower render.
    // 32 is fast, 128 is high quality.
    float stepSize = maxDistance / static_cast<float>(globalFog.stepCount); 
    glm::vec3 step = glm::normalize(ray.direction) * stepSize;

    // Param 2: Absorption (sigma_a). 
    // This is the "Thickness Multiplier".
    // 0.1 = Thin mist, 1.0 = Thick smoke.
    float sigma_a = globalFog.absorption; 

    float opticalDepth = 0.0f;
    glm::vec3 p = ray.origin;

    // Ray Marching Loop
    for (int i = 0; i < globalFog.stepCount; ++i) {
        p += step;
        float rho = getDensity(p); // Sample density at this point
        opticalDepth += sigma_a * rho * stepSize;
    }

    // Beer-Lambert Law: T = exp(-opticalDepth)
    return expf(-opticalDepth); 
}

glm::vec3 PhongModel(const Ray &ray, const Hit &hit)
{
  glm::vec3 color(0.0);
  Material mat = hit.object->getMaterial();

  for (int light_num = 0; light_num < lights.size(); light_num++)
  {
    Light *currentLight = lights[light_num];
    glm::vec3 light_direction = glm::normalize(currentLight->position - hit.intersection);
    float spotFactor = 1.0f;

    // --- SHADOW CALCULATION START ---
    float shadowFactor = 0.0f; // 0.0 = In Shadow, 1.0 = Fully Lit

    if (currentLight->radius > 0.0f)
    {
      // === SOFT SHADOWS (Area Light) ===
      int shadow_samples = 16;
      float visible_samples = 0.0f;

      for (int s = 0; s < shadow_samples; ++s)
      {
        // 1. Randomize point on light source
        glm::vec3 random_point = glm::vec3(random_range(-1, 1), random_range(-1, 1), random_range(-1, 1));
        random_point = glm::normalize(random_point) * currentLight->radius;
        glm::vec3 samplePos = currentLight->position + random_point;

        // 2. Setup Ray
        glm::vec3 sampleDir = glm::normalize(samplePos - hit.intersection);
        Ray shadowRay(hit.intersection + sampleDir * TOLERANCE, sampleDir);
        float distToLight = glm::distance(samplePos, hit.intersection);

        // 3. Trace Shadow Ray
        float light_transmission = 1.0f; // Start with full light

        for (int obj_num = 0; obj_num < objects.size(); ++obj_num)
        {
          Hit shadow_hit = objects[obj_num]->intersect(shadowRay);

          if (shadow_hit.hit && shadow_hit.distance < distToLight)
          {
            Material objMat = objects[obj_num]->getMaterial();

            // Ignore Emissive objects (the light itself)
            if (objMat.type == EMISSIVE)
              continue;

            if (objMat.krefract > 0.0f)
            {
              // --- GLASS SHADOW TRICK ---
              // Instead of ignoring glass, we say it blocks some light.
              // 0.6 means 60% of light gets through (Light Grey Shadow)
              // 0.0 means 0% gets through (Solid Black Shadow)
              light_transmission *= 0.6f;
            }
            else
            {
              // Hit a solid object (Wall/Mirror) -> Block light completely
              light_transmission = 0.0f;
              break; // Stop checking, we are blocked
            }
          }
        }
        visible_samples += light_transmission;
      }
      shadowFactor = visible_samples / (float)shadow_samples;
    }
    else
    {
      // === HARD SHADOWS (Point Light) ===
      Ray shadowRay(hit.intersection + light_direction * TOLERANCE, light_direction);
      float distToLight = glm::distance(currentLight->position, hit.intersection);
      float transmission = 1.0f;

      for (int obj_num = 0; obj_num < objects.size(); ++obj_num)
      {
        Hit shadow_hit = objects[obj_num]->intersect(shadowRay);
        if (shadow_hit.hit && shadow_hit.distance < distToLight)
        {
          if (objects[obj_num]->getMaterial().krefract > 0.0f)
          {
            transmission *= 0.6f; // Glass blocks partial light
          }
          else
          {
            transmission = 0.0f;
            break;
          }
        }
      }
      shadowFactor = transmission;
    }
    // --- SHADOW CALCULATION END ---

    // Spotlight check
    if (currentLight->cutoffAngleCos > 0.0f)
    {
      float dot = glm::dot(light_direction, -currentLight->direction);
      if (dot < currentLight->cutoffAngleCos)
        continue;
      spotFactor = pow(dot, currentLight->exp);
    }

    // Standard Phong Lighting (Diffuse + Specular)
    glm::vec3 reflected_direction = glm::reflect(-light_direction, hit.normal);
    float NdotL = glm::clamp(glm::dot(hit.normal, light_direction), 0.0f, 1.0f);
    float VdotR = glm::clamp(glm::dot(-ray.direction, reflected_direction), 0.0f, 1.0f);

    glm::vec3 diffuse_color = mat.diffuse;
    glm::vec3 diffuse = diffuse_color * glm::vec3(NdotL);
    glm::vec3 specular = mat.specular * glm::vec3(pow(VdotR, mat.shininess));

    float dist = glm::distance(currentLight->position, hit.intersection);
    float attenuation = 1.0f / (1.0f + 0.1f * dist + 0.01f * dist * dist);

    // Apply Shadow Factor
    color += currentLight->color * (diffuse + specular) * attenuation * shadowFactor * spotFactor;
  }
  color += ambient_light * mat.ambient;
  return color;
}

glm::vec3 trace_reflection(const Ray &ray, const Hit &hit, int depth)
{
  glm::vec3 reflectedDirection;
  reflectedDirection = glm::reflect(glm::normalize(ray.direction), glm::normalize(hit.normal));
  Ray reflectionRay(hit.intersection + TOLERANCE * reflectedDirection, reflectedDirection);
  return hit.object->getMaterial().kreflect * trace_ray(reflectionRay, depth + 1);
}

glm::vec3 trace_ray(const Ray &ray, int depth)
{
  Hit cHit;
  cHit.hit = false;
  cHit.distance = INFINITY;

  for (int k = 0; k < objects.size(); k++)
  {
    Hit hit = objects[k]->intersect(ray);
    if (hit.hit && hit.distance < cHit.distance)
      cHit = hit;
  }

  glm::vec3 black(0.0f);
  if (!cHit.hit)
    return black;

  Material mat = cHit.object->getMaterial();

  // Lights: return emissive term directly (no fog)
  if (mat.type == EMISSIVE)
    return mat.ambient;

  // Direct lighting (Phong or Ward)
  glm::vec3 direct_color(0.0f);
  if (mat.type == WARD)
    direct_color = WardModel(ray, cHit);
  else
    direct_color = PhongModel(ray, cHit);

  glm::vec3 result = direct_color;

  // Recursive reflection / refraction
  if (depth < MAX_RECURSION_DEPTH)
  {
    if (mat.krefract > 0.0f)
    {
      glm::vec3 incident = glm::normalize(ray.direction);
      glm::vec3 normal = glm::normalize(cHit.normal);

      float cosi = glm::clamp(glm::dot(-incident, normal), -1.0f, 1.0f);
      float eta_i = 1.0f;
      float eta_t = mat.refractIdx;

      if (cHit.isInsideObject)
        std::swap(eta_i, eta_t);

      float r0_term = (eta_i - eta_t) / (eta_i + eta_t);
      float R0 = r0_term * r0_term;
      float eta_ratio = eta_i / eta_t;
      float k = 1.0f - eta_ratio * eta_ratio * (1.0f - cosi * cosi);

      float R_fresnel;
      glm::vec3 refraction_color(0.0f);

      if (k < 0.0f)
      {
        R_fresnel = 1.0f; // total internal reflection
      }
      else
      {
        R_fresnel = R0 + (1.0f - R0) * pow(1.0f - cosi, 5.0f);

        glm::vec3 refracted = glm::refract(incident, normal, eta_ratio);
        glm::vec3 offset = -normal * TOLERANCE;
        Ray refr_ray(cHit.intersection + offset, refracted);
        refraction_color = trace_ray(refr_ray, depth + 1);
      }

      glm::vec3 reflected = glm::reflect(incident, normal);
      Ray refl_ray(cHit.intersection + normal * TOLERANCE, reflected);
      glm::vec3 reflection_color = trace_ray(refl_ray, depth + 1);

      result = R_fresnel * reflection_color + (1.0f - R_fresnel) * refraction_color;
      result += mat.transparency * direct_color;
    }
    else if (mat.kreflect > 0.0f)
    {
      glm::vec3 reflection_color = trace_reflection(ray, cHit, depth);
      result = (1.0f - mat.kreflect) * direct_color + mat.kreflect * reflection_color;
    }
  }

  // Volumetric fog along this ray segment
  float transmittance = computeTransmittance(ray, cHit.distance);
  glm::vec3 fogged = result * transmittance + globalFog.color * (1.0f - transmittance);
  return fogged;
}


void sceneDefinition(float t)
{
  fogBlobs.clear();

  // 1. Parametrize Global Settings
  globalFog.enabled = true;
  globalFog.color = glm::vec3(0.9f, 0.9f, 0.95f);  // White/Blue mist
  globalFog.absorption = 0.5f;                     // Medium thickness
  globalFog.stepCount = 64;

  // 2. Add Geometry
  // A huge blob centered below the floor.
  // Center: (0, -10, 10), Radius: 15.0, Peak Density: 0.5
  // The top of the gaussian curve will peek through the floor.
  fogBlobs.push_back({glm::vec3(0.0f, 4.9f, 10.0f), 1.2f * sqrt(t), 0.2f});

  // ============================
  // 1. MATERIALS
  // ============================

  // --- Ward Anisotropic (Brushed Gold) ---
  Material gold_ward;
  gold_ward.type = WARD;
  gold_ward.ambient = glm::vec3(0.2f, 0.15f, 0.05f);
  gold_ward.diffuse = glm::vec3(0.3f, 0.2f, 0.1f);
  gold_ward.specular = glm::vec3(0.9f, 0.8f, 0.4f);
  gold_ward.alphaX = 0.05f; // Sharp highlight
  gold_ward.alphaY = 0.6f;  // Blurry spread

  // --- Mirror ---
  Material mirror;
  mirror.type = PHONG;
  mirror.ambient = glm::vec3(0.0f);
  mirror.diffuse = glm::vec3(0.0f);
  mirror.specular = glm::vec3(0.0f);
  mirror.kreflect = 0.9f;
  mirror.shininess = 100.0f;

  // --- Matte Walls ---
  Material matte_white;
  matte_white.type = PHONG;
  matte_white.ambient = glm::vec3(0.1f);
  matte_white.diffuse = glm::vec3(0.9f);
  matte_white.specular = glm::vec3(0.0f);

  Material matte_purple;
  matte_purple.type = PHONG;
  matte_purple.ambient = glm::vec3(0.1f, 0.0f, 0.0f);
  matte_purple.diffuse = glm::vec3(157.0f / 255.0f, 48.0f / 255.0f, 176.0f / 255.0f);
  matte_purple.specular = glm::vec3(0.0f);

  Material matte_body;
  matte_body.type = PHONG;
  matte_body.ambient = glm::vec3(0.0f, 0.1f, 0.0f);
  // 207, 109, 152
  // 255, 162, 145
  matte_body.diffuse = glm::vec3(255.0f / 255.0f, 162.0f / 255.0f, 145.0f / 255.0f);
  matte_body.specular = glm::vec3(0.0f);

  Material matte_pinkish;
  matte_pinkish.type = PHONG;
  matte_pinkish.ambient = glm::vec3(0.0f, 0.0f, 0.1f);
  matte_pinkish.diffuse = glm::vec3(227.0f / 255.0f, 159.0f / 255.0f, 189.0f / 255.0f);
  matte_pinkish.specular = glm::vec3(0.0f);

  Material pink_em;
  pink_em.type = PHONG;
  pink_em.ambient = glm::vec3(0.3f);
  // pink_em.ambient = glm::vec3(227.0f/255.0f, 159.0f/255.0f, 189.0f/255.0f);
  // pink_em.ambient*=2;
  pink_em.diffuse = glm::vec3(227.0f / 255.0f, 159.0f / 255.0f, 189.0f / 255.0f);
  pink_em.specular = glm::vec3(0.1f);

  // --- Emissive Light Material (Fixes Black Hole) ---
  Material light_mat;
  light_mat.type = EMISSIVE; // Important: Matches the enum update
  light_mat.ambient = glm::vec3(0.5);
  light_mat.diffuse = glm::vec3(0.5*t);
  light_mat.specular = glm::vec3(0.0f);

  Material glass;
  glass.type = PHONG;
  glass.ambient = glm::vec3(0.0f);  // Glass does not have ambient color
  glass.diffuse = glm::vec3(0.0f);  // Glass is not diffuse (it is clear)
  glass.specular = glm::vec3(1.0f); // Bright white specular highlights
  glass.shininess = 100.0f;         // Sharp, small highlights

  glass.krefract = 1.0f;     // Enable refraction logic
  glass.refractIdx = 1.5f;   // Index of Refraction for Glass
  glass.transparency = 1.0f; // Ensure specular highlights (glints) are visible
  glass.kreflect = 0.0f;     // (Optional) Let Fresnel equations handle reflection
  // ============================
  // 2. GEOMETRY
  // ============================

  // Floor
  objects.push_back(new Plane(glm::vec3(0, -5, 0), glm::vec3(0, 1, 0), matte_pinkish));
  // Ceiling
  objects.push_back(new Plane(glm::vec3(0, 5, 0), glm::vec3(0, -1, 0), pink_em));
  // Back Wall
  objects.push_back(new Plane(glm::vec3(0, 0, 20), glm::vec3(0, 0, -1), matte_pinkish));
  // Left Wall (Red)
  objects.push_back(new Plane(glm::vec3(-6, 0, 0), glm::vec3(1, 0, 0), matte_purple));
  // Right Wall (Green)
  objects.push_back(new Plane(glm::vec3(6, 0, 0), glm::vec3(-1, 0, 0), matte_body));
  // front wall
  objects.push_back(new Plane(glm::vec3(0, 0, -5), glm::vec3(0, 0, 1), matte_body));

  // ============================
  // 3. OBJECTS
  // ============================

  glm::vec3 minB(-0.5f);
  glm::vec3 maxB(0.5f);
  Box *tallBox = new Box(minB, maxB, matte_white);

  // Transformation: Scale(3x6x3) -> Rotate(17 deg) -> Translate
  glm::mat4 tallTransform = glm::mat4(1.0f);
  tallTransform = glm::translate(tallTransform, glm::vec3(-2.0f, -2.0f, 12.0f));       // Position
  tallTransform = glm::rotate(tallTransform, glm::radians(17.0f), glm::vec3(0, 1, 0)); // Rotation Y
  tallTransform = glm::scale(tallTransform, glm::vec3(3.0f, 6.0f, 3.0f));              // Size

  tallBox->setTransformation(tallTransform);
  // objects.push_back(tallBox);

  // --- Short Box (Right) ---
  Box *shortBox = new Box(minB, maxB, matte_white);

  // Transformation: Scale(3x3x3) -> Rotate(-17 deg) -> Translate
  glm::mat4 shortTransform = glm::mat4(1.0f);
  shortTransform = glm::translate(shortTransform, glm::vec3(2.0f, -3.5f, 10.0f));         // Position
  shortTransform = glm::rotate(shortTransform, glm::radians(-17.0f + (-25.0f * t)), glm::vec3(0, 1, 0)); // Rotation Y
  shortTransform = glm::scale(shortTransform, glm::vec3(3.0f, 3.0f, 3.0f));               // Size

  shortBox->setTransformation(shortTransform);
  objects.push_back(shortBox);

  // Left: Ward Sphere
  objects.push_back(new Sphere(2.0f, glm::vec3(-3.0f, -3.0f, 8.0f), gold_ward));

  // Right: Mirror Sphere
  objects.push_back(new Sphere(1.5f, glm::vec3(3.0f, 1.0f, 14.0f), glass));
  objects.push_back(new Sphere(1.5f, glm::vec3(2.0f, -0.7f, 10.0f), glass));

  // Center Back: Blue Sphere
  objects.push_back(new Sphere(2.5f, glm::vec3(-1.5f + t, -3.5f, 16.0f), mirror));

  // ============================
  // 4. LIGHTING
  // ============================

  // --- A. Main Key Light (The Animated Hero Light) ---
  // Moved to 4.9f so it sits INSIDE the room (below ceiling)
  glm::vec3 mainLightPos = glm::vec3(0, 4.9f, 10.0f);

  // Animate intensity slightly (pulsing effect)
  float intensity = 0.6f + 0.1f * sin(t * 2.0f);
  glm::vec3 mainColor = glm::vec3(1.0f, 0.95f, 0.9f) * intensity;

  // Add visual sphere for main light
  objects.push_back(new Sphere(1.5f, mainLightPos, light_mat));
  // Add physical light (Radius 1.5 = Soft Shadows)
  lights.push_back(new Light(mainLightPos, mainColor, 1.5f));

  // --- B. Corner Fill Lights (Static) ---
  // We use radius 0.0f (Hard Shadows) for these to keep rendering FAST.
  // We use dim colors (0.2 or 0.3) so they don't overpower the main light.

  // struct CornerLight {
  //   glm::vec3 pos;
  //   glm::vec3 color;
  // };

  // vector< CornerLight > corners = {
  //     // Front Left: Subtle Cyan (Cool)
  //     {glm::vec3(-5.5f, 4.5f, 2.0f), glm::vec3(0.0f, 0.2f, 0.2f)},

  //     // Front Right: Subtle Orange (Warm)
  //     {glm::vec3(5.5f, 4.5f, 2.0f), glm::vec3(0.2f, 0.15f, 0.0f)},

  //     // Back Left:  Subtle Purple (Mystery)
  //     {glm::vec3(-5.5f, 4.5f, 18.0f), glm::vec3(0.15f, 0.0f, 0.2f)},

  //     // Back Right: Low White (Fill)
  //     {glm::vec3(5.5f, 4.5f, 18.0f), glm::vec3(0.1f, 0.1f, 0.1f)}};

  // for (const auto& L : corners) {
  //   // 1. Add a small visual bulb (so we see the source)
  //   objects.push_back(new Sphere(0.3f, L.pos, light_mat));

  //   // 2. Add the actual light source
  //   // NOTICE: Radius is 0.0f! This is crucial for speed.
  //   lights.push_back(new Light(L.pos, L.color, 0.0f));
  // }
}

glm::vec3 toneMapping(glm::vec3 color)
{
  float exposure = 1.8f;
  color *= exposure;

  // ACES approximation, this gives the best vibrant and saturated picture,
  // cinematic-like look
  color = (color * (2.51f * color + 0.03f)) /
          (color * (2.43f * color + 0.59f) + 0.14f);
  // reinhard
  // color = color / (color + 1.0f);

  color = pow(color, glm::vec3(1.0f / 2.2f));

  // float avg = (color.x + color.y + color.z) / 3.0f;
  // color = glm::vec3(avg);

  // color = glm::clamp(color, glm::vec3(0.0f), glm::vec3(1.0f));
  color = glm::clamp(color, glm::vec3(0.0f), glm::vec3(1.0f));
  return color;
}

atomic<int> pixels_rendered(0);

void printProgress(int totalPixels)
{
  while (pixels_rendered < totalPixels)
  {
    float progress = (float)pixels_rendered / totalPixels * 100.0f;
    cout << "\rRendering: " << progress << "% (" << pixels_rendered << "/" << totalPixels << ")" << flush;
    this_thread::sleep_for(chrono::milliseconds(500));
  }
  cout << "\rRendering: 100% (" << totalPixels << "/" << totalPixels << ")" << endl;
}

void renderTile(Image &img, int sx, int fx, int sy, int fy, float X, float Y, float s)
{
  float dz = 1;
  glm::vec3 cameraOrigin(0, 0, 0);

  for (int i = sx; i < fx; i++)
  {
    for (int j = sy; j < fy; j++)
    { // Corrected: change j < fy to j < fy
      glm::vec3 pixel_color(0.0f);

      for (int k = 0; k < AA_SAMPLES; ++k)
      {
        // ... (AA jitter for direction calculation remains here)
        float jitter_x = random_range(-s / 2, s / 2);
        float jitter_y = random_range(-s / 2, s / 2);

        float dx = X + i * s + s / 2 + jitter_x;
        float dy = Y - j * s - s / 2 + jitter_y;

        // 1. Calculate the focal point in the scene
        glm::vec3 rayDirectionAtPinhole(dx, dy, dz);
        glm::vec3 focalPoint = cameraOrigin + glm::normalize(rayDirectionAtPinhole) * FOCAL_DISTANCE;

        // 2. Randomize the ray origin (Aperture)
        glm::vec3 aperture_sample = random_in_unit_disk(APERTURE_RADIUS);
        // Note: The aperture is on the Z=0 plane since camera is at (0, 0, 0) looking down Z.
        glm::vec3 rayOrigin = cameraOrigin + aperture_sample;

        // 3. New ray direction targets the focal point
        glm::vec3 direction = glm::normalize(focalPoint - rayOrigin);

        Ray ray(rayOrigin, direction);
        pixel_color += trace_ray(ray, 0);
      }

      // Average and tone map...
      pixel_color /= (float)AA_SAMPLES;
      img.setPixel(i, j, toneMapping(pixel_color));

      ++pixels_rendered;
    }
  }
}

#include <iomanip>      // for std::setw
#include <filesystem>   // REQUIRED: for creating folders (C++17)
#include <sys/stat.h>  
bool directoryExists(const std::string& path) {
    struct stat info;
    // stat returns 0 if the path exists
    if (stat(path.c_str(), &info) != 0) {
        return false;
    }
    // Check if the path is a directory
    return (info.st_mode & S_IFDIR);
}

int main(int argc, const char *argv[]) {
    // === ANIMATION CONFIGURATION ===
    int fps = 24;
    int durationSec = 2;
    int totalFrames = fps * durationSec;
    string outputFolder = "renders"; // Name of the folder
    // ===============================

    // 1. Create the folder if it doesn't exist
    if (!directoryExists(outputFolder)) {
        mkdir(outputFolder.c_str(), 0777);
        cout << "Created folder: " << outputFolder << endl;
    }

    int multiplier = 2;
    int width = multiplier * 1024;
    int height = multiplier * 768;
    float fov = 90;

    Image image(width, height);

    float s = 2 * tan(0.5 * fov / 180 * M_PI) / width;
    float X = -s * width / 2;
    float Y = s * height / 2;

    unsigned int cores = thread::hardware_concurrency();
    if (cores == 0) cores = 4;

    cout << "Starting Render: " << totalFrames << " frames -> " << outputFolder << "/" << endl;

    for (int frame = 0; frame < totalFrames; ++frame) {
        
        float t = (float)frame / fps;
        
        // Clean up previous frame
        for (Object *obj : objects) delete obj;
        objects.clear();
        for (Light *light : lights) delete light;
        lights.clear();
        pixels_rendered = 0; 

        sceneDefinition(t); 

        cout << "Rendering Frame " << frame + 1 << " / " << totalFrames << endl;

        vector<thread> threads;
        int wPerCore = width / cores;

        for (int i = 0; i < cores; ++i) {
            int start = i * wPerCore;
            int finish = (i == cores - 1) ? width : (i + 1) * wPerCore;
            threads.emplace_back(renderTile, ref(image), start, finish, 0, height, X, Y, s);
        }

        for (auto &thread : threads) thread.join();

        // 2. Construct path: "renders/frame_000.ppm"
        stringstream ss;
        ss << outputFolder << "/frame_" << setfill('0') << setw(3) << frame << ".ppm";
        
        image.writeImage(ss.str().c_str());
    }

    // Cleanup final frame objects
    for (Object *obj : objects) delete obj;
    for (Light *light : lights) delete light;

    return 0;
}