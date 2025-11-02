/**
@file main.cpp
*/

#include <cmath>
#include <ctime>
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>
#include <thread>
#include <mutex>
#include <atomic>

#include "glm/glm.hpp"
#include "glm/gtx/transform.hpp"
#include "Image.h"
#include "Material.h"

using namespace std;

#define TOLERANCE        1e-4
#define WITHOUT_NORMALS  0
#define WITH_NORMALS     1
#define WITH_CUSTOM_FACE 2

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

class Triangle;
class BoundingBox {
public:
    glm::vec3 min;
    glm::vec3 max;
    
    BoundingBox() : min(glm::vec3(INFINITY)), max(glm::vec3(-INFINITY)) {}
    BoundingBox(glm::vec3 min, glm::vec3 max) : min(min), max(max) {}
    
    void expand(const glm::vec3& point) {
        min = glm::min(min, point);
        max = glm::max(max, point);
    }
    
    void expand(const BoundingBox& other) {
        min = glm::min(min, other.min);
        max = glm::max(max, other.max);
    }
    
    bool intersect(const Ray& ray) const {
        // Fast Ray-AABB intersection
        glm::vec3 invDir = 1.0f / ray.direction;
        glm::vec3 t1 = (min - ray.origin) * invDir;
        glm::vec3 t2 = (max - ray.origin) * invDir;
        
        glm::vec3 tmin = glm::min(t1, t2);
        glm::vec3 tmax = glm::max(t1, t2);
        
        float t_min = glm::max(glm::max(tmin.x, tmin.y), tmin.z);
        float t_max = glm::min(glm::min(tmax.x, tmax.y), tmax.z);
        
        return t_max >= glm::max(t_min, 0.0f);
    }
    
    float getSurfaceArea() const {
        glm::vec3 extent = max - min;
        return 2.0f * (extent.x * extent.y + extent.x * extent.z + extent.y * extent.z);
    }
    
    glm::vec3 getCenter() const {
        return (min + max) * 0.5f;
    }
};


class BVHNode {
public:
    BoundingBox bbox;
    BVHNode* left = nullptr;
    BVHNode* right = nullptr;
    vector<Triangle*> triangles;  // Only leaf nodes have triangles
    
    bool isLeaf() const { return left == nullptr && right == nullptr; }
    
    ~BVHNode() {
        delete left;
        delete right;
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
  glm::vec3 getNormal() { return normal; }
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
  ~Cone() {
    delete plane;
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
  vector<glm::vec3> n;
  bool smoothShading = false;

 public:
  Triangle(vector<glm::vec3> vertices, Material material) {
    v1 = vertices[0];
    v2 = vertices[1];
    v3 = vertices[2];
    glm::vec3 normal = glm::normalize(glm::cross(v2 - v1, v3 - v1));
    plane = new Plane(vertices[0], normal, material);
    this->material = material;
  }
  Triangle(vector<glm::vec3> vertices, vector<glm::vec3> normals, Material material) {
    v1 = vertices[0];
    v2 = vertices[1];
    v3 = vertices[2];
    glm::vec3 normal = glm::normalize(glm::cross(v2 - v1, v3 - v1));
    n = normals;
    smoothShading = true;
    plane = new Plane(vertices[0], normal, material);
    this->material = material;
  }
  void setSmooth(bool s) { smoothShading = s; }
  glm::vec3 getV1() const { return v1; }
  glm::vec3 getV2() const { return v2; }
  glm::vec3 getV3() const { return v3; }
  ~Triangle() {
    delete plane;
  }


Hit intersect(Ray ray) {
    Hit hit;
    hit.hit = false;
    hit.distance = INFINITY;

    /*
    // Plane intersection
    // y(t) = o + td;
    // <y(t) - v1, normal> = 0
    // <o - v1 + td, normal> = 0
    // <o - v1, normal> + <td, normal> = 0
    // t*<d, normal> = <v1 - o, normal>
    // t = <v1 - o, normal> / <d, normal>
    // float denominator = glm::dot(ray.direction, normal);

    Hit planeHit = plane->intersect(ray);
    if (!planeHit.hit) return hit;

    glm::vec3 A = v2 - v1;
    glm::vec3 B = v3 - v1;
    glm::vec3 C = planeHit.intersection - v1;

    float dot_AA = glm::dot(A, A);   
    float dot_BB = glm::dot(B, B);   
    float dot_AB = glm::dot(A, B);  
    float dot_CA = glm::dot(C, A);   
    float dot_CB = glm::dot(C, B);   

    // Compute the determinant (related to parallelogram area)
    // area = ||A x B||
    // area^2 = ||A x B||^2
    // area^2 = ||A x B|| * ||A x B||
    // Lagrange's identity: ||A||^2 * ||B||^2 - ||A * B||^2 = ||A X B||,
    // therefore area^2 (i.e. parallelogram area squared a.k.a.
    // determinant) = ||A x B||^2 = (A*A) * (B*B) - (A*B)*(A*B)
    float det = dot_AA * dot_BB - dot_AB * dot_AB;

    if (fabs(det) < TOLERANCE) { return hit; }

    // p = v1 + u(v2-v1) + v(v3 - v1)
    // p - v1 = uA + vB
    // C = uA + vB
    // CA = uAA + vBA - dot A on both sides
    // CB = uAB + vBB - dot B on both sides
	  // Matrix form:
	  // [AA BA] [u] - [CA]
	  // [AB BB] [v] - [CB]
    // Using Cramer's rule
    float u = (dot_CA * dot_BB - dot_CB * dot_AB) / det;
    float v = (dot_AA * dot_CB - dot_CA * dot_AB) / det;
    float y = 1.0f - u - v;

    if (u >= -TOLERANCE && v >= -TOLERANCE && y >= -TOLERANCE &&
        u <= 1.0f + TOLERANCE && v <= 1.0f + TOLERANCE &&
        y <= 1.0f + TOLERANCE) {
      if (fabs(u + v + y - 1.0f) < TOLERANCE * 3.0f) {
    */

    // Moller-Trumbore algorithm, more numerically stable
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

        if (smoothShading) {
          if (n.size() != 3) {
            cerr << "Size of normals is: " << n.size() << endl;
            exit(EXIT_FAILURE);
          }
          
          float w = 1.0f - (u + v);
          hit.normal = glm::normalize(n[0] * w + n[1] * u + n[2] * v);
        } else {
          hit.normal = glm::normalize(glm::cross(edge1, edge2));
        }
        hit.object = this;
    }
    
    return hit;
}
};

class Figure : public Object {
 private:
  vector< Triangle* > meshes;
  int mode;
  BVHNode* bvhRoot = nullptr;

 public:
  Figure(Material material, int m) {
    setMaterial(material);
    mode = m;
    meshes.reserve(10e3);
  }

  ~Figure() {
    delete bvhRoot;
    for (Triangle* triangle : meshes) {
        delete triangle;
    }
    meshes.clear();
  }

  void buildBVH() {
    if (meshes.empty()) return;

    vector< Triangle* > triangles = meshes;  // Copy for building
    bvhRoot = buildBVHNode(triangles, 0, triangles.size(), 0);
    cout << "BVH built for figure with " << meshes.size() << " triangles"
         << endl;
  }

  void addMesh(Triangle* tr) {
    tr->setMaterial(this->material);
    meshes.push_back(tr);
  }

  int getMode() { return mode; }
  void setSmooth(bool s) {
    for (auto tr : meshes) {
      tr->setSmooth(s);
    }
  }

  Hit intersect(Ray ray) {
    Hit closest_hit;
    closest_hit.distance = INFINITY;
    closest_hit.hit = false;

    glm::vec3 localOrigin = glm::vec3(inverseTransformationMatrix * glm::vec4(ray.origin, 1.0f));
    glm::vec3 localDirection = glm::vec3(inverseTransformationMatrix * glm::vec4(ray.direction, 0.0f));
    localDirection = glm::normalize(localDirection);
    Ray localRay(localOrigin, localDirection);

    // Use BVH if available, otherwise fall back to brute force
    if (bvhRoot) {
      closest_hit = intersectBVH(bvhRoot, localRay);
    } else {
      // Brute force fallback
      for (Triangle* triangle : meshes) {
        Hit hit = triangle->intersect(localRay);
        if (hit.hit && hit.distance < closest_hit.distance) {
          closest_hit = hit;
          closest_hit.object = this;
        }
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

 private:
  BoundingBox computeTriangleBBox(Triangle* triangle) {
    BoundingBox bbox;
    bbox.expand(triangle->getV1());
    bbox.expand(triangle->getV2());
    bbox.expand(triangle->getV3());
    return bbox;
  }

  BVHNode* buildBVHNode(vector< Triangle* >& triangles, int start, int end,
                        int depth) {
    BVHNode* node = new BVHNode();

    // Compute bounding box for all triangles in this node
    for (int i = start; i < end; i++) {
      node->bbox.expand(computeTriangleBBox(triangles[i]));
    }

    int triangleCount = end - start;

    // Leaf node condition: few triangles or max depth
    if (triangleCount <= 4 || depth >= 20) {
      node->triangles.insert(node->triangles.end(), triangles.begin() + start,
                             triangles.begin() + end);
      return node;
    }

    // Choose split axis (longest axis)
    glm::vec3 extent = node->bbox.max - node->bbox.min;
    int axis = (extent.x > extent.y && extent.x > extent.z) ? 0
               : (extent.y > extent.z)                      ? 1
                                                            : 2;

    // Sort triangles along the chosen axis
    auto comparator = [this, axis](Triangle* a, Triangle* b) {
      return computeTriangleBBox(a).getCenter()[axis] <
             computeTriangleBBox(b).getCenter()[axis];
    };

    int mid = start + (end - start) / 2;
    std::nth_element(triangles.begin() + start, triangles.begin() + mid,
                     triangles.begin() + end, comparator);

    // Recursively build child nodes
    node->left = buildBVHNode(triangles, start, mid, depth + 1);
    node->right = buildBVHNode(triangles, mid, end, depth + 1);

    return node;
  }

  Hit intersectBVH(BVHNode* node, const Ray& localRay) {
    if (!node->bbox.intersect(localRay)) {
      Hit miss;
      miss.hit = false;
      return miss;
    }

    if (node->isLeaf()) {
      // Check all triangles in leaf node
      Hit closest_hit;
      closest_hit.distance = INFINITY;
      closest_hit.hit = false;

      for (Triangle* triangle : node->triangles) {
        Hit hit = triangle->intersect(localRay);
        if (hit.hit && hit.distance < closest_hit.distance) {
          closest_hit = hit;
        }
      }
      return closest_hit;
    }

    // Check both children
    Hit hit_left = intersectBVH(node->left, localRay);
    Hit hit_right = intersectBVH(node->right, localRay);

    if (!hit_left.hit) return hit_right;
    if (!hit_right.hit) return hit_left;

    return (hit_left.distance < hit_right.distance) ? hit_left : hit_right;
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

struct FaceData {
    int v, t, n;
};

FaceData parseFaceToken(const string& faceData, Figure* figure) {
    FaceData result = {-1, -1, -1};
    string processed = faceData;

    switch (figure->getMode()) {
      case WITH_CUSTOM_FACE: {
        // Format: v/t/n - replace all slashes with spaces
        replace(processed.begin(), processed.end(), '/', ' ');
        break;
      }
      case WITH_NORMALS: {
        // Format: v//n - replace double slash with space
        size_t pos = processed.find("//");
        if (pos != string::npos) {
          processed.replace(pos, 2, " ");
        }
        break;
      }
      default:
        // Format: v - no processing needed
        break;
    }

    istringstream faceStream(processed);
    
    switch (figure->getMode()) {
        case WITH_CUSTOM_FACE:
            faceStream >> result.v >> result.t >> result.n;
            break;
        case WITH_NORMALS:
            faceStream >> result.v >> result.n;
            break;
        case WITHOUT_NORMALS:
            faceStream >> result.v;
            break;
    }
    
    return result;
}

bool validateFaceData(const vector<int>& vIndices, const vector<int>& nIndices, Figure* figure) {
    if (vIndices.size() != 3) return false;
    
    switch (figure->getMode()) {
        case WITHOUT_NORMALS:
            return true;
        case WITH_NORMALS:
        case WITH_CUSTOM_FACE:
            return nIndices.size() == 3;
        default:
            return false;
    }
}

vector<glm::vec3> getVertices(const vector<int>& vIndices, const vector<glm::vec3>& vertices) {
    return {
        vertices[vIndices[0]],
        vertices[vIndices[1]], 
        vertices[vIndices[2]]
    };
}

vector<glm::vec3> getNormals(const vector<int>& nIndices, const vector<glm::vec3>& normals) {
    return {
        normals[nIndices[0]],
        normals[nIndices[1]],
        normals[nIndices[2]]
    };
}

void processFace(Figure* figure, istringstream& iss, 
                vector<glm::vec3>& normals, vector<glm::vec3>& vertices, 
                Material mat) {
    
    vector<int> vIndices;
    vector<int> nIndices;
    string faceData;
    
    while (iss >> faceData) {
        FaceData data = parseFaceToken(faceData, figure);
        
        if (data.v != -1) vIndices.push_back(data.v);
        if (data.n != -1) nIndices.push_back(data.n);
    }
    
    if (!validateFaceData(vIndices, nIndices, figure)) {
        cerr << "Warning: #indices - " << vIndices.size();
        cerr << " #normals - " << nIndices.size() << endl;
        return;
    }
    
    vector<glm::vec3> triangleVertices = getVertices(vIndices, vertices);
    
    if (figure->getMode() == WITHOUT_NORMALS) {
        figure->addMesh(new Triangle(triangleVertices, mat));
    } else {
        vector<glm::vec3> triangleNormals = getNormals(nIndices, normals);
        figure->addMesh(new Triangle(triangleVertices, triangleNormals, mat));
    }
}

void loadMesh(Figure* figure, string filename, Material material) {
	string line;
	ifstream file(filename);

	if (!file.is_open()) {
		cerr << "Error: could not open the file: " << filename << endl;
		exit(1);
	}

	vector<glm::vec3> vertices(1, glm::vec3(0.0f));
	vector<glm::vec3> normals(1, glm::vec3(0.0f));
  int faces = 0;

	while (getline(file, line)) {
		if (line.empty() || line[0] == '#') continue;

		istringstream iss(line);
		string token;
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

	cout << "Finished loading mesh from: " << filename << endl;
	cout << "Vertices: " << vertices.size() << "; ";
	cout << "Normals: " << normals.size() << "; ";
	cout << "Polygons: " << faces << endl;
	file.close();
}

void sceneDefinition() {
  Material silver;
  silver.ambient = glm::vec3(0.8f, 0.8f, 0.9f);
  silver.diffuse = glm::vec3(0.7f, 0.7f, 0.8f);
  silver.specular = glm::vec3(1.0f, 1.0f, 1.0f);
  silver.shininess = 100.0f;

  Figure* human = new Figure(silver, WITH_CUSTOM_FACE);
  loadMesh(human, string("meshes/human.obj"), silver);
  human->buildBVH();
  glm::mat4 translationMatrix = glm::translate(glm::vec3(0, -3, 6));
  glm::mat4 scalingMatrix = glm::scale(glm::vec3(0.035f));
  glm::mat4 rotX = glm::rotate(glm::radians(-90.0f), glm::vec3(1, 0, 0));
  glm::mat4 rotY = glm::rotate(glm::radians(0.0f), glm::vec3(0, 1, 0));
  glm::mat4 rotZ = glm::rotate(glm::radians(180.0f), glm::vec3(0, 0, 1));
  glm::mat4 rotateMatrix = rotY * rotX * rotZ;
  human->setTransformation(translationMatrix * rotateMatrix * scalingMatrix);
  objects.push_back(human);

  Figure* bunny = new Figure(silver, WITHOUT_NORMALS);
  loadMesh(bunny, string("meshes/bunny.obj"), silver);
  bunny->buildBVH();
  translationMatrix = glm::translate(glm::vec3(-4, -2, 7));
  scalingMatrix = glm::scale(glm::vec3(1.0f));
  bunny->setTransformation(translationMatrix * scalingMatrix);
  objects.push_back(bunny);

  Figure* bunny_with_normals = new Figure(silver, WITH_NORMALS);
  loadMesh(bunny_with_normals, string("meshes/bunny_with_normals.obj"), silver);
  bunny_with_normals->buildBVH();
  translationMatrix = glm::translate(glm::vec3(4, -2, 6));
  scalingMatrix = glm::scale(glm::vec3(1.0f));
  glm::mat4 rotateBunnyY = glm::rotate(glm::radians(90.0f), glm::vec3(0, 1, 0));
  bunny_with_normals->setTransformation(translationMatrix * rotateBunnyY * scalingMatrix);
  objects.push_back(bunny_with_normals);

  Figure* armadillo_with_normals = new Figure(silver, WITH_NORMALS);
  loadMesh(armadillo_with_normals, string("meshes/armadillo_with_normals.obj"), silver);
  armadillo_with_normals->buildBVH();
  translationMatrix = glm::translate(glm::vec3(-3, -2, 9));
  scalingMatrix = glm::scale(glm::vec3(1.0f));
  //glm::mat4 rotatearmadilloY = glm::rotate(glm::radians(90.0f), glm::vec3(0, 1, 0));
  //armadillo_with_normals->setTransformation(translationMatrix * rotatearmadilloY * scalingMatrix);
  armadillo_with_normals->setTransformation(translationMatrix * scalingMatrix);
  objects.push_back(armadillo_with_normals);

  Figure* lucy_with_normals = new Figure(silver, WITH_NORMALS);
  loadMesh(lucy_with_normals, string("meshes/lucy_with_normals.obj"), silver);
  lucy_with_normals->buildBVH();
  translationMatrix = glm::translate(glm::vec3(3, -2, 9));
  scalingMatrix = glm::scale(glm::vec3(1.0f));
  //glm::mat4 rotatelucyY = glm::rotate(glm::radians(90.0f), glm::vec3(0, 1, 0));
  //lucy_with_normals->setTransformation(translationMatrix * rotatelucyY * scalingMatrix);
  lucy_with_normals->setTransformation(translationMatrix * scalingMatrix);
  objects.push_back(lucy_with_normals);

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

// Thread-safe counter for progress tracking
atomic<int> pixels_rendered(0);

void renderTile(int startX, int endX, int startY, int endY, Image& image, 
                float s, float X, float Y, int width, int height) {
    for (int i = startX; i < endX; i++) {
        for (int j = startY; j < endY; j++) {
            float dx = X + i * s + s / 2;
            float dy = Y - j * s - s / 2;
            float dz = 1;

            glm::vec3 origin(0, 0, 0);
            glm::vec3 direction(dx, dy, dz);
            direction = glm::normalize(direction);

            Ray ray(origin, direction);
            image.setPixel(i, j, toneMapping(trace_ray(ray)));
            
            pixels_rendered++;
        }
    }
}

void printProgress(int totalPixels) {
    while (pixels_rendered < totalPixels) {
        float progress = (float)pixels_rendered / totalPixels * 100.0f;
        cout << "\rRendering: " << progress << "% (" << pixels_rendered << "/" << totalPixels << ")" << flush;
        this_thread::sleep_for(chrono::milliseconds(500));
    }
    cout << "\rRendering: 100% (" << totalPixels << "/" << totalPixels << ")" << endl;
}

int main(int argc, const char* argv[]) {
    auto t0 = chrono::high_resolution_clock::now();

    int width = 1024;
    int height = 768;
    float fov = 90;

    sceneDefinition();

    Image image(width, height);
    
    float s = 2 * tan(0.5 * fov / 180 * M_PI) / width;
    float X = -s * width / 2;
    float Y = s * height / 2;

    // Determine number of threads
    unsigned int numThreads = thread::hardware_concurrency();
    if (numThreads == 0) numThreads = 4; // Fallback
    cout << "Using " << numThreads << " threads" << endl;

    // Split by columns (better cache locality)
    vector<thread> threads;
    int tileWidth = width / numThreads;
    
    // Start progress thread
    thread progressThread(printProgress, width * height);

    // Launch render threads
    for (int t = 0; t < numThreads; t++) {
        int startX = t * tileWidth;
        int endX = (t == numThreads - 1) ? width : (t + 1) * tileWidth;

        threads.emplace_back(renderTile, startX, endX, 0, height, ref(image), s, X, Y, width, height);
    }

    // Wait for all threads to complete
    for (auto& thread : threads) {
        thread.join();
    }
    
    // Wait for progress thread to finish
    progressThread.join();

    auto t1 = chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0);
    float seconds = duration.count() / 1000.0f;
    float fps = 1.0f / seconds;

    std::cout << "It took " << seconds << " seconds to render the image." << std::endl;
    std::cout << "I could render at " << fps << " frames per second." << std::endl;

    if (argc == 2) {
        image.writeImage(argv[1]);
    } else {
        image.writeImage("./result.ppm");
    }

    // Cleanup
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