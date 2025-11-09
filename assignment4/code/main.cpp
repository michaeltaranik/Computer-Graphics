/**
        @file main.cpp
        */

#include <atomic>
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

/**
        Class representing a single ray.
        */
class Ray {
 public:
  glm::vec3 origin;     ///< Origin of the ray
  glm::vec3 direction;  ///< Direction of the ray
  /**
          Contructor of the ray
          @param origin Origin of the ray
          @param direction Direction of the ray
          */
  Ray(glm::vec3 origin, glm::vec3 direction)
      : origin(origin), direction(direction) {}
};

class Object;
glm::vec3 toneMapping(glm::vec3);
glm::vec3 trace_ray(const Ray &ray, int depth);

/**
        Structure representing the even of hitting an object
        */
struct Hit {
  bool hit;          ///< Boolean indicating whether there was or there was no
                     ///< intersection with an object
  glm::vec3 normal;  ///< Normal vector of the intersected object at the
                     ///< intersection point
  glm::vec3 intersection;  ///< Point of Intersection
  float distance;  ///< Distance from the origin of the ray to the intersection
                   ///< point
  Object *object;  ///< A pointer to the intersected object
};

/**
        General class for the object
        */
class Object {
 protected:
  glm::mat4
      transformationMatrix;  ///< Matrix representing the transformation from
                             ///< the local to the global coordinate system
  glm::mat4 inverseTransformationMatrix;  ///< Matrix representing the
                                          ///< transformation from the global to
                                          ///< the local coordinate system
  glm::mat4 normalMatrix;  ///< Matrix for transforming normal vectors from the
                           ///< local to the global coordinate system

 public:
  glm::vec3 color;    ///< Color of the object
  Material material;  ///< Structure describing the material of the object
  /** A function computing an intersection, which returns the structure Hit */
  virtual Hit intersect(Ray ray) = 0;
  virtual ~Object() = default;

  /** Function that returns the material struct of the object*/
  Material getMaterial() { return material; }
  /** Function that set the material
          @param material A structure describing the material of the object
          */
  void setMaterial(Material material) { this->material = material; }
  /** Functions for setting up all the transformation matrices
          @param matrix The matrix representing the transformation of the object
     in the global coordinates */
  void setTransformation(glm::mat4 matrix) {
    transformationMatrix = matrix;

    /* ----- Exercise 2 ---------
             Set the two remaining matrices
             inverseTransformationMatrix =
             normalMatrix =
             */
    inverseTransformationMatrix = glm::inverse(matrix);
    normalMatrix = glm::transpose(inverseTransformationMatrix);
  }
};

/**
        Implementation of the class Object for sphere shape.
        */
class Sphere : public Object {
 private:
  float radius;      ///< Radius of the sphere
  glm::vec3 center;  ///< Center of the sphere

 public:
  /**
          The constructor of the sphere
          @param radius Radius of the sphere
          @param center Center of the sphere
          @param color Color of the sphere
          */
  Sphere(float radius, glm::vec3 center, glm::vec3 color)
      : radius(radius), center(center) {
    this->color = color;
  }
  Sphere(float radius, glm::vec3 center, Material material)
      : radius(radius), center(center) {
    this->material = material;
  }
  /** Implementation of the intersection function*/
  Hit intersect(Ray ray) {
    glm::vec3 c = center - ray.origin;

    float cdotc = glm::dot(c, c);
    float cdotd = glm::dot(c, ray.direction);

    Hit hit;

    float D = 0;
    if (cdotc > cdotd * cdotd) {
      D = sqrt(cdotc - cdotd * cdotd);
    }
    if (D <= radius) {
      hit.hit = true;
      float t1 = cdotd - sqrt(radius * radius - D * D);
      float t2 = cdotd + sqrt(radius * radius - D * D);

      float t = t1;
      if (t < 0) t = t2;
      if (t < 0) {
        hit.hit = false;
        return hit;
      }

      hit.intersection = ray.origin + t * ray.direction;
      hit.normal = glm::normalize(hit.intersection - center);
      hit.distance = glm::distance(ray.origin, hit.intersection);
      hit.object = this;
    } else {
      hit.hit = false;
    }
    return hit;
  }
};

class Plane : public Object {
 private:
  glm::vec3 normal;
  glm::vec3 point;

 public:
  Plane(glm::vec3 point, glm::vec3 normal) : point(point), normal(normal) {}
  Plane(glm::vec3 point, glm::vec3 normal, Material material)
      : point(point), normal(normal) {
    this->material = material;
  }
  void setNormal(glm::vec3 n) { normal = n; }
  Hit intersect(Ray ray) {
    Hit hit;
    hit.hit = false;

    float denominator = glm::dot(normal, ray.direction);
    if (abs(denominator) >= TOLERANCE) {
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
      if (t < 0) return hit;
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

class Cone : public Object {
 private:
  Plane *plane;

  Hit coneSurfaceIntersect(Ray localRay, glm::vec3 worldOrigin) {
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
    if (D < 0) return hit;
    float t1 = (-B - glm::sqrt(D)) / (2.0f * A);
    float t2 = (-B + glm::sqrt(D)) / (2.0f * A);
    float t = -1.0f;

    glm::vec3 localIntersection;
    if (t1 > TOLERANCE) {
      localIntersection = localOrigin + t1 * localDirection;
      if (localIntersection.y >= -1.0f && localIntersection.y <= 0.0f) {
        t = t1;
      }
    }

    if (t2 > TOLERANCE) {
      if (t < 0 || t2 < t) {
        localIntersection = localOrigin + t2 * localDirection;
        if (localIntersection.y >= -1.0f && localIntersection.y <= 0.0f) {
          t = t2;
        }
      }
    }

    if (t < 0) return hit;

    glm::vec3 localNormal = glm::normalize(glm::vec3( localIntersection.x, -localIntersection.y, localIntersection.z));
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
  Cone(Material material) {
    this->material = material;
    // Base plane for cone that is pointing down(x: 0, y: -1, z: 0)
    plane = new Plane(glm::vec3(0, -1, 0), glm::vec3(0, -1, 0), material);
  }
  ~Cone() {
    delete plane;
  }

  Hit intersect(Ray ray) {
    glm::vec3 localOrigin = glm::vec3(inverseTransformationMatrix * glm::vec4(ray.origin, 1.0f));
    glm::vec3 localDirection = glm::vec3(inverseTransformationMatrix * glm::vec4(ray.direction, 0.0f));
    localDirection = glm::normalize(localDirection);
    Ray localRay(localOrigin, localDirection);
    // broke down into two different functions for base and surface, since it
    // makes more sense to reuse the code from Plane class
    Hit surfaceHit = coneSurfaceIntersect(localRay, ray.origin);
    Hit baseHit = plane->intersect(localRay);
    if (baseHit.hit) {
      float t = baseHit.distance;
      glm::vec3 localBasePoint = baseHit.intersection;
      float radius2 = localBasePoint.x * localBasePoint.x +
                      localBasePoint.z * localBasePoint.z;
      // checking whether the point is within the circle on the plane
      if (radius2 < 1.0f) {
        baseHit.intersection = glm::vec3(transformationMatrix * glm::vec4(localBasePoint, 1.0f));
        baseHit.normal = glm::vec3(normalMatrix * glm::vec4(baseHit.normal, 0.0f));
        baseHit.normal = glm::normalize(baseHit.normal);
        baseHit.distance = glm::distance(baseHit.intersection, ray.origin);
      } else {
        baseHit.hit = false;
      }
    }
    if (surfaceHit.hit && baseHit.hit) {
      // return the closest hit
      if (surfaceHit.distance < baseHit.distance) return surfaceHit;
      return baseHit;
    } else if (surfaceHit.hit)
      return surfaceHit;
    return baseHit;
  }
};

/**
Light class
*/
class Light {
 public:
  glm::vec3 position;  ///< Position of the light source
  glm::vec3 color;     ///< Color/intentisty of the light source
  Light(glm::vec3 position) : position(position) { color = glm::vec3(1.0); }
  Light(glm::vec3 position, glm::vec3 color)
      : position(position), color(color) {}
};

vector< Light * > lights;  ///< A list of lights in the scene
// reduced it a bit
glm::vec3 ambient_light(0.01f);
vector< Object * > objects;  ///< A list of all objects in the scene

/** Function for computing color of an object according to the Phong Model
        @param point A point belonging to the object for which the color is
   computer
        @param normal A normal vector the the point
        @param view_direction A normalized direction from the point to the
   viewer/camera
        @param material A material structure representing the material of the
   object
        */
glm::vec3 PhongModel(glm::vec3 point, glm::vec3 normal,
                     glm::vec3 view_direction, Material material) {
  glm::vec3 color(0.0);
  for (int light_num = 0; light_num < lights.size(); light_num++) {
    glm::vec3 light_direction = glm::normalize(lights[light_num]->position - point);
    bool in_shadow = false;

    Ray shadowRay(point + normal * TOLERANCE, light_direction);
    for (int obj_num = 0; obj_num < objects.size(); ++obj_num) {
      Hit shadow_hit = objects[obj_num]->intersect(shadowRay);
      if (shadow_hit.hit &&
          shadow_hit.distance < glm::distance(lights[light_num]->position, point)) {
          in_shadow = true;
          break;
      }
    }

    if (in_shadow) continue;

    glm::vec3 reflected_direction = glm::reflect(-light_direction, normal);
    float NdotL = glm::clamp(glm::dot(normal, light_direction), 0.0f, 1.0f);
    float VdotR = glm::clamp(glm::dot(view_direction, reflected_direction), 0.0f, 1.0f);

    glm::vec3 diffuse_color = material.diffuse;
    glm::vec3 diffuse = diffuse_color * glm::vec3(NdotL);
    glm::vec3 specular = material.specular * glm::vec3(pow(VdotR, material.shininess));

    float dist = glm::distance(lights[light_num]->position, point);
    float attenuation = 1.0f / (1.0f + 0.1f * dist + 0.01f * dist * dist);
    color += lights[light_num]->color * (diffuse + specular) * attenuation;
  }
  color += ambient_light * material.ambient;
  return color;
}

/**
        Functions that computes a color along the ray
        @param ray Ray that should be traced through the scene
        @return Color at the intersection point
        */
glm::vec3 trace_ray(const Ray &ray, int depth) {
  Hit closest_hit;

  closest_hit.hit = false;
  closest_hit.distance = INFINITY;

  for (int k = 0; k < objects.size(); k++) {
    Hit hit = objects[k]->intersect(ray);
    if (hit.hit == true && hit.distance < closest_hit.distance)
      closest_hit = hit;
  }

  glm::vec3 color(0.0);
  if (closest_hit.hit) {
    if (depth <= MAX_RECURSION_DEPTH && closest_hit.object->getMaterial().isReflective) {
      glm::vec3 reflectedDirection = glm::reflect(glm::normalize(ray.direction), glm::normalize(closest_hit.normal));
      Ray reflectionRay(closest_hit.intersection + TOLERANCE * reflectedDirection, reflectedDirection);
      color = trace_ray(reflectionRay, depth + 1);
    } else {
      color = PhongModel(closest_hit.intersection, closest_hit.normal, glm::normalize(-ray.direction), closest_hit.object->getMaterial());
    }
  } else {
    color = glm::vec3(0.0, 0.0, 0.0);
  }
  return color;
}
/**
        Function defining the scene
        */
void sceneDefinition() {
  Material emerald_green;
  emerald_green.ambient = glm::vec3(0.1f, 0.6f, 0.3f);
  emerald_green.diffuse = glm::vec3(0.05f, 0.5f, 0.2f);
  emerald_green.specular = glm::vec3(0.4f, 0.9f, 0.6f);
  emerald_green.shininess = 30.0f;

  Material ruby_red;
  ruby_red.ambient = glm::vec3(0.08f, 0.1f, 0.02f);
  ruby_red.diffuse = glm::vec3(0.88f, 0.05f, 0.15f);
  ruby_red.diffuse /= glm::vec3(5.0f);
  ruby_red.specular = glm::vec3(0.1f, 0.05f, 0.06f);
  ruby_red.shininess = 8.0f;

  Material sapphire_blue;
  sapphire_blue.ambient = glm::vec3(0.1f, 0.2f, 0.15f);
  sapphire_blue.diffuse = glm::vec3(0.05f, 0.2f, 0.6f);
  sapphire_blue.specular = glm::vec3(0.5f, 0.6f, 1.0f);
  sapphire_blue.shininess = 150.0f;

  Material gold;
  gold.ambient = glm::vec3(0.09f, 0.07f, 0.01f);
  gold.diffuse = glm::vec3(0.98f, 0.6f, 0.05f);
  gold.diffuse /= glm::vec3(4.0f);
  gold.specular = glm::vec3(0.9f);
  gold.shininess = 100.0f;

  Material silver;
  silver.ambient = glm::vec3(0.8f, 0.8f, 0.9f);
  silver.diffuse = glm::vec3(0.7f, 0.7f, 0.8f);
  silver.specular = glm::vec3(1.0f, 1.0f, 1.0f);
  silver.shininess = 100.0f;

  Material warm_floor;
  warm_floor.ambient = glm::vec3(0.095f, 0.092f, 0.085f);
  warm_floor.diffuse = glm::vec3(0.95f, 0.92f, 0.95f);
  warm_floor.diffuse /= glm::vec3(2.0f);
  warm_floor.specular = glm::vec3(0.9f);
  warm_floor.shininess = 10.0f;

  Material sky_backdrop;
  sky_backdrop.ambient = glm::vec3(0.6f, 0.8f, 1.0f);
  sky_backdrop.diffuse = glm::vec3(0.5f, 0.7f, 0.9f);
  sky_backdrop.specular = glm::vec3(0.1f);
  sky_backdrop.shininess = 1.0f;

  Material red_specular;
  red_specular.ambient = glm::vec3(0.0f, 0.1f, 0.1f);
  red_specular.diffuse = glm::vec3(0.99f, 0.1f, 0.1f);
  red_specular.diffuse /= glm::vec3(2.0f);
  red_specular.specular = glm::vec3(0.1f);
  red_specular.shininess = 10.0;

  Material blue_specular;
  blue_specular.ambient = glm::vec3(0.07f, 0.07f, 0.0f);
  blue_specular.diffuse = glm::vec3(0.5f, 0.5f, 0.5f);
  blue_specular.diffuse /= glm::vec3(3.0f);
  blue_specular.specular = glm::vec3(0.9);
  blue_specular.shininess = 50.0;
  blue_specular.isReflective = true;

  Material floor_material;
  floor_material.ambient = glm::vec3(1.0f, 1.0f, 1.0f);
  floor_material.diffuse = glm::vec3(1.0f, 1.0f, 1.0f);
  floor_material.specular = glm::vec3(0.6);
  floor_material.shininess = 10.0;

  Material back_wall_material;
  back_wall_material.ambient = glm::vec3(0.02f, 0.05f, 0.02f);
  back_wall_material.diffuse = glm::vec3(0.0f, 0.94f, 0.16f);
  back_wall_material.diffuse /= glm::vec3(5.0f);
  back_wall_material.specular = glm::vec3(0.1f);
  back_wall_material.shininess = 1.0f;

  objects.push_back(new Sphere(1.0, glm::vec3(1, -2, 8), blue_specular));
  objects.push_back(new Sphere(0.5, glm::vec3(-1, -2.5, 6), red_specular));
  objects.push_back(new Sphere(2.0, glm::vec3(-3,-1, 8), sky_backdrop));
  // objects.push_back(new Sphere(1.0, glm::vec3(2,-2,6), green_diffuse));

  // Planes
  // left wall: from -15 to 15 (1, 0, 0)
  objects.push_back(
      new Plane(glm::vec3(-15, 0, 0), glm::vec3(1, 0, 0), ruby_red));
  // right wall: from 15 to -15 (-1, 0, 0)
  objects.push_back(
      new Plane(glm::vec3(15, 0, 0), glm::vec3(-1, 0, 0), sapphire_blue));
  // back wall
  objects.push_back(
      new Plane(glm::vec3(0, 0, 30), glm::vec3(0, 0, -1), back_wall_material));
  // floor
  objects.push_back(
      new Plane(glm::vec3(0, -3, 0), glm::vec3(0, 1, 0), warm_floor));
  // ceiling, not visible
  objects.push_back(
      new Plane(glm::vec3(0, 27, 0), glm::vec3(0, -1, 0), emerald_green));
  // front wall, not visible
  objects.push_back(
      new Plane(glm::vec3(0, 0, -0.01), glm::vec3(0, 0, 1), floor_material));

  // Cones
  // Yellow cone
  Material yellow_specular;
  yellow_specular.ambient = glm::vec3(0.014f, 0.011f, 0.01f);
  yellow_specular.diffuse = glm::vec3(1.0f, 1.0f, 0.1f);
  yellow_specular.specular = glm::vec3(0.8f);
  yellow_specular.shininess = 100.0f;
  Cone *yellowCone = new Cone(gold);
  glm::mat4 translation_yellow =
      glm::translate(glm::mat4(1.0f), glm::vec3(5, 9, 14));
  glm::mat4 scaling_yellow = glm::scale(glm::mat4(1.0f), glm::vec3(3, 12, 3));
  glm::mat4 transformation_yellow = translation_yellow * scaling_yellow;
  yellowCone->setTransformation(transformation_yellow);
  objects.push_back(yellowCone);

  // Green cone
  Cone *greenCone = new Cone(emerald_green);
  glm::mat4 translation_green =
      glm::translate(glm::mat4(1.0f), glm::vec3(6, -3, 7));
  float coneHeight = 3.0f;
  float coneBase = 1.0f;
  glm::mat4 scaling_green = glm::scale(glm::mat4(1.0f), glm::vec3(coneBase, coneHeight, coneBase));
  glm::mat4 rotation_green = glm::rotate(glm::mat4(1.0f), glm::radians(-90.0f) - atan(coneBase/coneHeight), glm::vec3(0, 0, 1));
  //glm::mat4 rotation_green = glm::rotate(glm::mat4(1.0f), glm::radians(-112.5f), glm::vec3(0, 0, 1));
  glm::mat4 transformation_green = translation_green * rotation_green * scaling_green;
  greenCone->setTransformation(transformation_green);
  objects.push_back(greenCone);

  lights.push_back(new Light(glm::vec3(0, 26, 5), glm::vec3(0.7f)));
  lights.push_back(new Light(glm::vec3(0, 1, 12), glm::vec3(0.55f)));
  lights.push_back(new Light(glm::vec3(0, 5, 1), glm::vec3(0.63f)));
}

glm::vec3 toneMapping(glm::vec3 color) {
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

  //color = glm::clamp(color, glm::vec3(0.0f), glm::vec3(1.0f));
  return color;
}

atomic<int> pixels_rendered(0);

void printProgress(int totalPixels) {
    while (pixels_rendered < totalPixels) {
        float progress = (float)pixels_rendered / totalPixels * 100.0f;
        cout << "\rRendering: " << progress << "% (" << pixels_rendered << "/" << totalPixels << ")" << flush;
        this_thread::sleep_for(chrono::milliseconds(500));
    }
    cout << "\rRendering: 100% (" << totalPixels << "/" << totalPixels << ")" << endl;
}

void renderTile(Image& img, int sx, int fx, int sy, int fy, float X, float Y, float s) {
  for (int i = sx; i < fx; i++) {
    for (int j = sy; j < fy; j++) {
      float dx = X + i * s + s / 2;
      float dy = Y - j * s - s / 2;
      float dz = 1;

      glm::vec3 origin(0, 0, 0);
      glm::vec3 direction(dx, dy, dz);
      direction = glm::normalize(direction);

      Ray ray(origin, direction);
      img.setPixel(i, j, toneMapping(trace_ray(ray, 0)));
      ++pixels_rendered;
    }
  }
}

int main(int argc, const char *argv[]) {
  auto t0 = chrono::high_resolution_clock::now();
  int multiplier = 2;

  int width = multiplier*1024;  // width of the image
  int height = multiplier*768;  // height of the image
  float fov = 90;    // field of view

  sceneDefinition();  // Let's define a scene

  Image image(width, height);  // Create an image where we will store the result

  float s = 2 * tan(0.5 * fov / 180 * M_PI) / width;
  float X = -s * width / 2;
  float Y = s * height / 2;
  cout << "s: " << s << endl;
  cout << "X: " << X << endl;
  cout << "Y: " << Y << endl;

  unsigned int cores = thread::hardware_concurrency();
  if (cores == 0) { cores = 4; }
  cout << "Using " << cores << " threads\n";

  vector<thread> threads;
  int wPerCore = width / cores;

  thread progressThread(printProgress, width * height);
  for (int i = 0; i < cores; ++i) {
    int start = i * wPerCore;
    int finish = (i == cores - 1) ? width : (i + 1) * wPerCore;

    threads.emplace_back(renderTile, ref(image), start, finish, 0, height, X, Y, s);
  }

  for (auto& thread: threads) {
    thread.join();
  }

  progressThread.join();

  auto t1 = chrono::high_resolution_clock::now();
  auto duration = chrono::duration_cast<std::chrono::milliseconds>(t1 - t0);
  float seconds = duration.count() / 1000.0f;
  float fps = 1.0f / seconds;

  std::cout << "It took " << seconds << " seconds to render the image." << std::endl;
  std::cout << "I could render at " << fps << " frames per second." << std::endl;

  if (argc == 2) {
    image.writeImage(argv[1]);
  } else {
    image.writeImage("./result.ppm");
  }

  for (Object* obj: objects) {
    delete obj;
  }
  objects.clear();
  for (Light* light: lights) {
    delete light;
  }
  lights.clear();
  return 0;
}
