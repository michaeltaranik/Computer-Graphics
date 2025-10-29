/**
@file main.cpp
*/

#include <cmath>
#include <ctime>
#include <fstream>
#include <iostream>
#include <vector>

#include "Image.h"
#include "Material.h"
#include "glm/glm.hpp"
#include "glm/gtx/transform.hpp"

using namespace std;

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
  bool onEdge;
};

/**
 General class for the object
 */
class Object {
 public:
  glm::vec3 color;    ///< Color of the object
  Material material;  ///< Structure describing the material of the object
  /** A function computing an intersection, which returns the structure Hit */
  virtual Hit intersect(Ray ray) = 0;

  /** Function that returns the material struct of the object*/
  Material getMaterial() { return material; }
  /** Function that set the material
   @param material A structure describing the material of the object
  */
  void setMaterial(Material material) { this->material = material; }
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
    Hit hit;
    hit.hit = false;
    // hit.onEdge = false;

    /* ------------------ Exercise 1 -------------------

    Place for your code: ray-sphere intersection. Remember to set all the fields
    of the hit structure:

     hit.intersection =
     hit.normal =
     hit.distance =
     hit.object = this;

    ------------------------------------------------- */
    // computing the vector directed to the center
    glm::vec3 center_vector = center - ray.origin;
    // the signed distance from the ray origin to the closest approach point on
    // the ray to the sphere's center
    float aProj = glm::dot(center_vector, ray.direction);
    // the squared distance from the sphere's center to the closest approach
    // point
    float d2 = glm::dot(center_vector, center_vector) - aProj * aProj;
    float r2 = radius * radius;

    // out of sphere
    if (d2 > r2) {
      // std::cout << "hit false: d2 > r2\n";
      return hit;
    }

    // the delta (difference between the intersection and radius (center))
    // deltaB is the half-chord length inside the sphere from the closest
    // approach point
    float deltaB = sqrt(r2 - d2);
    // possible points of intersections
    float t1 = aProj - deltaB;
    float t2 = aProj + deltaB;

    float t = (t1 > 0) ? t1 : t2;

    // if sphere is behind the camera/eyes
    if (t < 0) {
      // std::cout << "hit false: t < 0\n";
      return hit;
    }

    // float edge_threshhold = 0.1f;
    // if (deltaB <= edge_threshhold) {
    //   // cout << "edge!\n";
    //   hit.onEdge = true;
    // }

    // y(t) = o + td
    hit.intersection = ray.origin + t * ray.direction;
    // represents a vector pointing to the point of intersection, essential for
    // lighting/shading calculations
    hit.normal = glm::normalize(hit.intersection - center);
    hit.distance = t;
    hit.object = this;
    hit.hit = true;
    // std::cout << "hit true\n";
    return hit;
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
glm::vec3 ambient_light(0.1, 0.1, 0.1);
vector< Object * > objects;  ///< A list of all objects in the scene

/** Function for computing color of an object according to the Phong Model
 @param point A point belonging to the object for which the color is computed
 @param normal A normal vector the the point
 @param view_direction A normalized direction from the point to the
 viewer/camera
 @param material A material structure representing the material of the object
*/
glm::vec3 PhongModel(glm::vec3 point, glm::vec3 normal,
                     glm::vec3 view_direction, Material material) {
  glm::vec3 color(0.0);

  /* ------------------Excercise 3--------------------

   Phong model.
   Your code should implement a loop over all the lightsourses in the array
   lights and agredate the contribution of each of them to the final color of
   the object. Outside of the loop add also the ambient component from
   ambient_light.

   ------------------------------------------------- */

  // cout << "before: " << color.x << ' ' << color.y << ' ' << color.z << endl;
  // color += material.ambient * ambient_light * 15.0f;
  // Setting the default color given the material ambient factor and the default
  // ambient light (how well ambient is illuminated)
  // RHOa * Ia
  color += material.ambient * ambient_light;
  // cout << "after: " << color.x << ' ' << color.y << ' ' << color.z << endl;

  // cout << material.ambient.x << ' ' << material.ambient.y << ' '
  //      << material.ambient.z << endl;
  // cout << ambient_light.x << ' ' << ambient_light.y << ' ' << ambient_light.z
  //      << endl;

  // Multiple light sources are added together, but each individual light
  // contribution is multiplied by the material to simulate absorption (more
  // like matte) and reflection (specular)

  for (Light *&light : lights) {
    // tried to implement attenuation
    // float attenuation = 1.0f / (glm::distance(light->position, point));
    // normalized 'ray' direction hitting the object
    glm::vec3 light_dir = glm::normalize(light->position - point);

    // Diffuse component, responsible for natural color
    // we negated the ray direction before to compute the dot product now and
    // see whether it is 1 (vectors point to the same direction) or -1 (totally
    // opposite directions) or 0 (orthogonal case)
    float diffuse = glm::max(glm::dot(normal, light_dir), 0.0f);
    // material.diffuse - material response, how well it reflects or bad it
    // absorbs
    // diffuse - geometry factor, should be more illuminated if it facing the
    // light source more; lambert's cosine law
    // light->color - incoming light, the intensity of the light source
    // Id = RHOd * cos(phi) * I = RHOd * <n, l> * I
    color += material.diffuse * diffuse * light->color;
    // color += material.diffuse * diffuse * light->color * attenuation;

    // Specular component, responsible for mirroring light, shiny highlight
    // l|| - projection of l onto n
    // l|| = (n * l)n
    // l = l|| + l| (l orthogonal)
    // l| (orthogonal) = l - l||
    // r (reflection vector) = l|| - l| = l|| - (l - l||)
    // r = l|| - l + l|| = 2l|| - l = 2*(n*l)n - l
    glm::vec3 reflection_direction =
        (2.0f * glm::dot(normal, light_dir) * normal) - light_dir;
    // (cos(alpha))^k
    float specular =
        glm::pow(glm::max(glm::dot(view_direction, reflection_direction), 0.0f),
                 material.shininess);
    // Is = RHOs * (cos(alpha))^k * I
    color += material.specular * specular * light->color;
    // color += material.specular * specular * light->color * attenuation;
  }

  // The final color has to be clamped so the values do not go beyond 0 and 1.
  color = glm::clamp(color, glm::vec3(0.0), glm::vec3(1.0));
  return color;
}

/**
 Functions that computes a color along the ray
 @param ray Ray that should be traced through the scene
 @return Color at the intersection point
 */
glm::vec3 trace_ray(Ray ray) {
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
    /* ------------------Excercise 3--------------------

     Use the second line when you implement PhongModel function - Exercise 3

     ------------------------------------------------- */
    // color = closest_hit.object->color;
    // if (closest_hit.onEdge) {
    //   color = glm::vec3(1.0);
    // } else {
    color = PhongModel(closest_hit.intersection, closest_hit.normal,
                       glm::normalize(-ray.direction),
                       closest_hit.object->getMaterial());
    // }
  } else {
    color = glm::vec3(0.0, 0.0, 0.0);
    // 168, 50, 80
    // color = glm::vec3(0.25, 0.2, 0.31);
  }
  return color;
}
/**
 Function defining the scene
 */
void sceneDefinition() {
  Material blue_mat, red_mat, green_mat;
  blue_mat.ambient = glm::vec3(0.07, 0.07, 0.1);
  blue_mat.diffuse = glm::vec3(0.7, 0.7, 1.0);
  blue_mat.specular = glm::vec3(0.6, 0.6, 0.6);
  blue_mat.shininess = 100.0f;

  red_mat.ambient = glm::vec3(0.01, 0.03, 0.03);
  red_mat.diffuse = glm::vec3(1.0, 0.03, 0.03);
  red_mat.specular = glm::vec3(0.5, 0.5, 0.5);
  red_mat.shininess = 10.0f;

  green_mat.ambient = glm::vec3(0.07, 0.09, 0.07);
  green_mat.diffuse = glm::vec3(0.7, 0.9, 0.7);
  green_mat.specular = glm::vec3(0.0);
  green_mat.shininess = 0.0f;

  objects.push_back(
      new Sphere(1.0f, glm::vec3(1.0, -2.0, 8.0), blue_mat));  // blue
  objects.push_back(
      new Sphere(0.5f, glm::vec3(-1.0, -2.5, 6.0), red_mat));  // red
  objects.push_back(
      new Sphere(1.0, glm::vec3(2.0, -2.0, 6.0), green_mat));  // green

  /* ------------------Excercise 2--------------------
  Place for your code: additional sphere
  ------------------------------------------------- */

  /* ------------------Excercise 3--------------------

   Add here all the objects to the scene. Remember to add them using the
  constructor for the sphere with material structure. You will also need to
  define the materials. Example of adding one sphere:

   Material red_specular;
   red_specular.diffuse = glm::vec3(1.0f, 0.3f, 0.3f);
   red_specular.ambient = glm::vec3(0.01f, 0.03f, 0.03f);
   red_specular.specular = glm::vec3(0.5);
   red_specular.shininess = 10.0;

   objects.push_back(new Sphere(0.5, glm::vec3(-1,-2.5,6), red_specular));

   Remember also about adding some lights. For example a white light of
  intensity 0.4 and position in (0,26,5):

   lights.push_back(new Light(glm::vec3(0, 26, 5), glm::vec3(0.4)));

  ------------------------------------------------- */
  lights.push_back(new Light(glm::vec3(0, 26, 5), glm::vec3(0.4)));
  lights.push_back(new Light(glm::vec3(0, 1, 12), glm::vec3(0.4)));
  lights.push_back(new Light(glm::vec3(0, 5, 1), glm::vec3(0.4)));
}

int main(int argc, const char *argv[]) {
  clock_t t = clock();  // variable for keeping the time of the rendering

  int width = 4 * 1024;  // width of the image
  int height = 4 * 768;  // height of the image
  float fov = 90;        // field of view

  sceneDefinition();  // Let's define a scene

  Image image(width, height);  // Create an image where we will store the result

  /* ------------------ Exercise 1 -------------------

  Place for your code: Loop over pixels to form and traverse the rays through
  the scene

  ------------------------------------------------- */
  // glm::vec3 origin(0.3, 0.5, 0.5);
  glm::vec3 origin(0.0);
  float dz = 1.0f;
  float scale = tan(glm::radians(fov * 0.5f));
  float pixelSize = 2 * scale / width;
  float xLeft = -width * pixelSize / 2.0f;
  float yTop = height * pixelSize / 2.0f;
  // cout << "scale: " << scale << endl;
  // cout << "pixelSize: " << pixelSize << endl;
  // cout << "X: " << xLeft << endl;
  // cout << "Y: " << yTop << endl;

  for (int i = 0; i < width; i++)
    for (int j = 0; j < height; j++) {
      /* ------------------ Exercise 1 -------------------

      Place for your code: ray definition for pixel (i,j), ray traversal

       ------------------------------------------------- */

      // Definition of the ray
      // glm::vec3 origin(0, 0, 0);
      // glm::vec3 direction(?, ?, ?);               // fill in the correct
      // values direction = glm::normalize(direction);
      // Ray ray(origin, direction);  // ray traversal
      // image.setPixel(i, j, trace_ray(ray));

      float dx, dy;
      dx = xLeft + 0.5f * pixelSize + i * pixelSize;
      dy = yTop - (0.5f * pixelSize + j * pixelSize);
      glm::vec3 direction = glm::normalize(glm::vec3(dx, dy, dz) - origin);
      Ray ray(origin, direction);
      image.setPixel(i, j, trace_ray(ray));
    }

  t = clock() - t;
  cout << "It took " << ((float)t) / CLOCKS_PER_SEC
       << " seconds to render the image." << endl;
  cout << "I could render at " << (float)CLOCKS_PER_SEC / ((float)t)
       << " frames per second." << endl;

  // Writing the final results of the rendering
  if (argc == 2) {
    image.writeImage(argv[1]);
  } else {
    image.writeImage("./result.ppm");
  }

  return 0;
}
