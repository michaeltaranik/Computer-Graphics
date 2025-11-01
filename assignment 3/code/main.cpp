/**
@file main.cpp
*/

#include <cmath>
#include <ctime>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>

#include "glm/glm.hpp"
#include "glm/gtx/transform.hpp"
#include "Image.h"
#include "Material.h"

using namespace std;

#define TOLERANCE 1e-4

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
  Object* object;  ///< A pointer to the intersected object
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
  @param matrix The matrix representing the transformation of the object in the
  global coordinates */
  void setTransformation(glm::mat4 matrix) {
    transformationMatrix = matrix;

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
  Hit intersect(Ray ray) {
    Hit hit;
    hit.hit = false;

    float DdotN = glm::dot(ray.direction, normal);
    if (DdotN < 0) {
      float PdotN = glm::dot(point - ray.origin, normal);
      float t = PdotN / DdotN;

      if (t > 0) {
        hit.hit = true;
        hit.normal = normal;
        hit.distance = t;
        hit.object = this;
        hit.intersection = t * ray.direction + ray.origin;
        // Refactor
        // hit.hit = true;
        // hit.normal = glm::vec3(normalMatrix * glm::vec4(normal, 0.0f));
        // hit.normal = glm::normalize(hit.normal);
        // hit.distance = t;
        // hit.object = this;
        // hit.intersection = glm::vec3(transformationMatrix * glm::vec4(t *
        // ray.direction + ray.origin, 1.0f));
      }
    }

    return hit;
  }
};

class Cone : public Object {
 private:
  Plane* plane;

 public:
  Cone(Material material) {
    this->material = material;
    plane = new Plane(glm::vec3(0, 1, 0), glm::vec3(0.0, 1, 0));
  }
  Hit intersect(Ray ray) {
    Hit hit;
    hit.hit = false;

    glm::vec3 d = inverseTransformationMatrix * glm::vec4(ray.direction, 0.0);  // implicit cast to vec3
    glm::vec3 o = inverseTransformationMatrix * glm::vec4(ray.origin, 1.0);  // implicit cast to vec3
    d = glm::normalize(d);

    float a = d.x * d.x + d.z * d.z - d.y * d.y;
    float b = 2 * (d.x * o.x + d.z * o.z - d.y * o.y);
    float c = o.x * o.x + o.z * o.z - o.y * o.y;

    float delta = b * b - 4 * a * c;

    if (delta < 0) {
      return hit;
    }

    float t1 = (-b - sqrt(delta)) / (2 * a);
    float t2 = (-b + sqrt(delta)) / (2 * a);

    float t = t1;
    hit.intersection = o + t * d;
    if (t < 0 || hit.intersection.y > 1 || hit.intersection.y < 0) {
      t = t2;
      hit.intersection = o + t * d;
      if (t < 0 || hit.intersection.y > 1 || hit.intersection.y < 0) {
        return hit;
      }
    };

    hit.normal = glm::vec3(hit.intersection.x, -hit.intersection.y, hit.intersection.z);
    hit.normal = glm::normalize(hit.normal);

    Ray new_ray(o, d);
    Hit hit_plane = plane->intersect(new_ray);
    if (hit_plane.hit && hit_plane.distance < t &&
        length(hit_plane.intersection - glm::vec3(0, 1, 0)) <= 1.0) {
      hit.intersection = hit_plane.intersection;
      hit.normal = hit_plane.normal;
    }

    hit.hit = true;
    hit.object = this;
    hit.intersection = transformationMatrix * glm::vec4(hit.intersection, 1.0);  // implicit cast to vec3
    hit.normal = (normalMatrix * glm::vec4(hit.normal, 0.0));  // implicit cast to vec3
    hit.normal = glm::normalize(hit.normal);
    hit.distance = glm::length(hit.intersection - ray.origin);
    return hit;
  }
};

class Triangle : public Object {
 private:
  Plane* plane;
  glm::vec3 v1;
  glm::vec3 v2;
  glm::vec3 v3;

 public:
  Triangle(glm::vec3 p1, glm::vec3 p2, glm::vec3 p3, glm::vec3 normal, Material material) : v1(p1), v2(p2), v3(p3) {
    plane = new Plane(p1, normal, material);
    this->material = material;
  }

Hit intersect(Ray ray) {
    Hit hit;
    hit.hit = false;
    hit.distance = INFINITY;

    // Moller-Trumbore algorithm
    glm::vec3 edge1 = v2 - v1;
    glm::vec3 edge2 = v3 - v1;
    glm::vec3 h = glm::cross(ray.direction, edge2);
    float a = glm::dot(edge1, h);
    
    if (fabs(a) < TOLERANCE) return hit; // Ray parallel to triangle
    
    float f = 1.0f / a;
    glm::vec3 s = ray.origin - v1;
    float u = f * glm::dot(s, h);
    
    if (u < 0.0f || u > 1.0f) return hit;
    
    glm::vec3 q = glm::cross(s, edge1);
    float v = f * glm::dot(ray.direction, q);
    
    if (v < 0.0f || u + v > 1.0f) return hit;
    
    // At this stage, we can compute t to find out where the intersection point is on the line
    float t = f * glm::dot(edge2, q);
    
    if (t > TOLERANCE) {
        hit.hit = true;
        hit.distance = t;
        hit.intersection = ray.origin + t * ray.direction;
        hit.normal = glm::normalize(glm::cross(edge1, edge2));
        hit.object = this;
    }
    
    return hit;
}
};

class Figure : public Object {
 private:
  std::vector< Triangle* > meshes;
  bool smoothShading = false;

 public:
  Figure(Material material) {
    setMaterial(material);
    meshes.reserve(10e3);
  }

  ~Figure() {
    for (Triangle* triangle : meshes) {
        delete triangle;
    }
    meshes.clear();
}

  void addMesh(Triangle* tr) {
    tr->setMaterial(this->material);
    meshes.push_back(tr);
  }
  void setSmooth(bool smooth) { smoothShading = smooth; }
  bool getSmoothShading() { return smoothShading; }

  Hit intersect(Ray ray) {
    Hit closest_hit;
    closest_hit.distance = INFINITY;
    closest_hit.hit = false;

    glm::vec3 localOrigin = glm::vec3(inverseTransformationMatrix * glm::vec4(ray.origin, 1.0f));
    glm::vec3 localDirection = glm::vec3(inverseTransformationMatrix * glm::vec4(ray.direction, 0.0f));
    localDirection = glm::normalize(localDirection);
    Ray localRay(localOrigin, localDirection);

    for (Triangle* triangle : meshes) {
      Hit hit = triangle->intersect(localRay);
      if (hit.hit && hit.distance < closest_hit.distance) {
        closest_hit = hit;
        closest_hit.object = this;
      }
    }

    if (closest_hit.hit) {
        closest_hit.intersection = transformationMatrix * glm::vec4(closest_hit.intersection, 1.0);  // implicit cast to vec3
        closest_hit.normal = (normalMatrix * glm::vec4(closest_hit.normal, 0.0));  // implicit cast to vec3
        closest_hit.normal = glm::normalize(closest_hit.normal);
        closest_hit.distance = glm::length(closest_hit.intersection - ray.origin);
    }
    return closest_hit;
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

vector< Light* > lights;  ///< A list of lights in the scene
// glm::vec3 ambient_light(0.1,0.1,0.1);
//  new ambient light
glm::vec3 ambient_light(0.001, 0.001, 0.001);
vector< Object* > objects;  ///< A list of all objects in the scene

/** Function for computing color of an object according to the Phong Model
 @param point A point belonging to the object for which the color is computer
 @param normal A normal vector the the point
 @param view_direction A normalized direction from the point to the
 viewer/camera
 @param material A material structure representing the material of the object
*/
glm::vec3 PhongModel(glm::vec3 point, glm::vec3 normal,
                     glm::vec3 view_direction, Material material) {
  glm::vec3 color(0.0);
  for (int light_num = 0; light_num < lights.size(); light_num++) {
    glm::vec3 light_direction = glm::normalize(lights[light_num]->position - point);
    glm::vec3 reflected_direction = glm::reflect(-light_direction, normal);

    float NdotL = glm::clamp(glm::dot(normal, light_direction), 0.0f, 1.0f);
    float VdotR = glm::clamp(glm::dot(view_direction, reflected_direction), 0.0f, 1.0f);

    glm::vec3 diffuse_color = material.diffuse;
    glm::vec3 diffuse = diffuse_color * glm::vec3(NdotL);
    glm::vec3 specular = material.specular * glm::vec3(pow(VdotR, material.shininess));

    float r = glm::distance(point, lights[light_num]->position);
    r = max(r, 0.1f);
    color += lights[light_num]->color * (diffuse + specular) / r / r;
  }
  color += ambient_light * material.ambient;
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
    color = PhongModel(closest_hit.intersection, closest_hit.normal,
                       glm::normalize(-ray.direction),
                       closest_hit.object->getMaterial());
  } else {
    color = glm::vec3(0.0, 0.0, 0.0);
  }
  return color;
}

void processFace(Figure* figure, std::istringstream& iss, std::vector<glm::vec3> &normals, std::vector<glm::vec3> &vertices, Material mat) {
  std::string faceData;
  std::vector< int > vIndices;
  std::vector< int > nIndices;

  while (iss >> faceData) {
    std::replace(faceData.begin(), faceData.end(), '/', ' ');
    std::istringstream faceStream(faceData);

    int v, t, n;
    if (faceStream >> v >> t >> n) {
      vIndices.push_back(v);
      nIndices.push_back(n);
    }
  }

  if (vIndices.size() == 3 && nIndices.size() == 3) {
    glm::vec3 normal = (normals[nIndices[0]] + normals[nIndices[1]] + normals[nIndices[2]]) / 3.0f;
    figure->addMesh(new Triangle(vertices[vIndices[0]], vertices[vIndices[1]], vertices[vIndices[2]], normal, mat));
  } else {
    std::cerr << "Warning: #indices - " << vIndices.size();
    std::cerr << " #normals - " << nIndices.size() << endl;
  }
}

void loadMesh(Figure* figure, std::string filename, Material material) {
	std::string line;
	ifstream file(filename);

	if (!file.is_open()) {
		std::cerr << "Error: could not open the file: " << filename << endl;
		exit(1);
	}

	std::vector<glm::vec3> vertices(1, glm::vec3(0.0f));
	std::vector<glm::vec3> normals(1, glm::vec3(0.0f));
  int faces = 0;

	while (getline(file, line)) {
		if (line.empty() || line[0] == '#') continue;

		std::istringstream iss(line);
		std::string token;
		iss >> token;

		if (token == "v") {
			glm::vec3 vertex;
			iss >> vertex.x >> vertex.y >> vertex.z;
			vertices.push_back(vertex);
		} else if (token == "vn") {
			glm::vec3 normal;
			iss >> normal.x >> normal.y >> normal.z;
			normals.push_back(normal);
		} else if (token == "f") {
      processFace(figure, iss, normals, vertices, material);
      ++faces;
		} else if (token == "s") {
			bool smooth;
			iss >> smooth;
			figure->setSmooth(smooth);
		}
	}

	std::cout << "Finished loading mesh from: " << filename << endl;
	std::cout << "Vertices: " << vertices.size() << endl;
	std::cout << "Normals: " << normals.size() << endl;
	std::cout << "Polygons: " << faces << endl;
	file.close();
}

void sceneDefinition() {
  Material silver;
  silver.ambient = glm::vec3(0.8f, 0.8f, 0.9f);
  silver.diffuse = glm::vec3(0.7f, 0.7f, 0.8f);
  silver.specular = glm::vec3(1.0f, 1.0f, 1.0f);
  silver.shininess = 100.0f;

  Figure* female = new Figure(silver);
  loadMesh(female, std::string("meshes/Female_Demo.obj"), silver);
  glm::mat4 translationMatrix = glm::translate(glm::vec3(0, -3, 10));
  glm::mat4 scalingMatrix = glm::scale(glm::vec3(0.055f));
  glm::mat4 rotX = glm::rotate(glm::radians(-90.0f), glm::vec3(1, 0, 0));
  glm::mat4 rotY = glm::rotate(glm::radians(0.0f), glm::vec3(0, 1, 0));
  glm::mat4 rotZ = glm::rotate(glm::radians(180.0f), glm::vec3(0, 0, 1));
  glm::mat4 rotateMatrix = rotY * rotX * rotZ;
  female->setTransformation(translationMatrix * rotateMatrix * scalingMatrix);
  objects.push_back(female);

 // Figure* bunny = new Figure(silver);
 // loadMesh(bunny, std::string("meshes/Female_Demo.obj"), silver, false);
 // glm::mat4 translationMatrix = glm::translate(glm::vec3(0, 0, 11));
 // glm::mat4 scalingMatrix = glm::scale(glm::vec3(1.0f));
 // bunny->setTransformation(translationMatrix * scalingMatrix);
 // objects.push_back(bunny);

  //  glm::vec3 p1(-0.5f, -0.5f, 3.0f);
  //  glm::vec3 p2(0.5f, -0.5f, 3.0f);
  //  glm::vec3 p3(0.0f, 0.5f, 3.0f);
  //  glm::vec3 normal(0.0f, 0.0f, -1.0f);
  //
  //  Triangle* triangle = new Triangle(p1, p2, p3, normal, silver);
  //  triangle->setTransformation(translationMatrix * scalingMatrix);
  //  objects.push_back(triangle);

  Material green_diffuse;
  green_diffuse.ambient = glm::vec3(0.03f, 0.1f, 0.03f);
  green_diffuse.diffuse = glm::vec3(0.3f, 1.0f, 0.3f);

  lights.push_back(new Light(glm::vec3(0, 26, 5), glm::vec3(1.0, 1.0, 1.0)));
  lights.push_back(new Light(glm::vec3(0, 1, 12), glm::vec3(0.1)));
  lights.push_back(new Light(glm::vec3(0, 5, 1), glm::vec3(0.4)));

  Material red_diffuse;
  red_diffuse.ambient = glm::vec3(0.09f, 0.06f, 0.06f);
  red_diffuse.diffuse = glm::vec3(0.9f, 0.6f, 0.6f);

  Material blue_diffuse;
  blue_diffuse.ambient = glm::vec3(0.06f, 0.06f, 0.09f);
  blue_diffuse.diffuse = glm::vec3(0.6f, 0.6f, 0.9f);

  objects.push_back(new Plane(glm::vec3(0, -3, 0), glm::vec3(0.0, 1, 0)));
  objects.push_back( new Plane(glm::vec3(0, 1, 30), glm::vec3(0.0, 0.0, -1.0), green_diffuse));
  objects.push_back( new Plane(glm::vec3(-15, 1, 0), glm::vec3(1.0, 0.0, 0.0), red_diffuse));
  objects.push_back( new Plane(glm::vec3(15, 1, 0), glm::vec3(-1.0, 0.0, 0.0), blue_diffuse));
  objects.push_back(new Plane(glm::vec3(0, 27, 0), glm::vec3(0.0, -1, 0)));
  objects.push_back(new Plane(glm::vec3(0, 1, -0.01), glm::vec3(0.0, 0.0, 1.0), green_diffuse));
}

glm::vec3 toneMapping(glm::vec3 intensity) {
  float gamma = 1.0 / 2.0;
  float alpha = 12.0f;
  return glm::clamp(alpha * glm::pow(intensity, glm::vec3(gamma)), glm::vec3(0.0), glm::vec3(1.0));
}

int main(int argc, const char* argv[]) {
  clock_t t = clock();  // variable for keeping the time of the rendering

  int width = 1024/4;  // width of the image
  int height = 768/4;  // height of the image
  float fov = 90;    // field of view

  sceneDefinition();  // Let's define a scene

  Image image(width, height);  // Create an image where we will store the result
  vector< glm::vec3 > image_values(width * height);

  float s = 2 * tan(0.5 * fov / 180 * M_PI) / width;
  float X = -s * width / 2;
  float Y = s * height / 2;

  for (int i = 0; i < width; i++)
    for (int j = 0; j < height; j++) {
      float dx = X + i * s + s / 2;
      float dy = Y - j * s - s / 2;
      float dz = 1;

      glm::vec3 origin(0, 0, 0);
      glm::vec3 direction(dx, dy, dz);
      direction = glm::normalize(direction);

      Ray ray(origin, direction);
      image.setPixel(i, j, toneMapping(trace_ray(ray)));
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

  for (Object* obj : objects) {
    delete obj;
  }
  objects.clear();
  for (Light* light : lights) {
    delete light;
  }
  lights.clear();

  return 0;
}
