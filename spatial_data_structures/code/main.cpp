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

const float TOLERANCE = 2e-7;

#define WITHOUT_NORMALS     0
#define WITH_NORMALS        1
#define WITH_CUSTOM_FACE    2

class Object;
class Light;
vector< Light* > lights;  
glm::vec3 ambient_light(0.001, 0.001, 0.001);
vector< Object* > objects;

class Ray {
 public:
  glm::vec3 origin;     
  glm::vec3 direction;  
  Ray(glm::vec3 origin, glm::vec3 direction) : origin(origin), direction(direction) {}
  Ray(float dx, float dy, float dz) : origin(0, 0, 0), direction(dx, dy, dz) {
    direction = glm::normalize(direction);
  }
};

class Light {
 public:
  glm::vec3 position; 
  glm::vec3 color;  
  Light(glm::vec3 position) : position(position) { color = glm::vec3(1.0); }
  Light(glm::vec3 position, glm::vec3 color) : position(position), color(color) {}
};


struct Hit {
  bool hit;         
  glm::vec3 normal; 
  glm::vec3 intersection;  
  float distance;  
  Object* object;  
};

class Object {
 protected:
  glm::mat4 transformationMatrix;  
  glm::mat4 inverseTransformationMatrix; 
  glm::mat4 normalMatrix;  

 public:
  glm::vec3 color;    
  Material material;  

  virtual Hit intersect(Ray ray) = 0;
  virtual ~Object() = default;

  Material getMaterial() { return material; }

  void setMaterial(Material material) { this->material = material; }
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
    
    glm::vec3 getCenterDoubled() const {
        return min + max;
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
    vector<Triangle*> triangles;  // only leaf nodes have triangles
    
    bool isLeaf() const { return left == nullptr && right == nullptr; }
    
    ~BVHNode() {
        delete left;
        delete right;
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

class Triangle : public Object {
 private:
  Plane* plane;
  glm::vec3 v1;
  glm::vec3 v2;
  glm::vec3 v3;
  vector<glm::vec3> n;

 public:
  Triangle(vector<glm::vec3> &vertices, Material material) {
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
    plane = new Plane(vertices[0], normal, material);
    this->material = material;
  }
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

    // Moller-Trumbore algorithm, more numerically stable
    glm::vec3 edge1 = v2 - v1;
    glm::vec3 edge2 = v3 - v1;
    glm::vec3 h = glm::cross(ray.direction, edge2);
    float a = glm::dot(edge1, h);

    if (fabs(a) < TOLERANCE) return hit;  // ray parallel to triangle

    float f = 1.0f / a;
    glm::vec3 s = ray.origin - v1;
    float u = f * glm::dot(s, h);

    if (u < 0.0f || u > 1.0f) return hit;

    glm::vec3 q = glm::cross(s, edge1);
    float v = f * glm::dot(ray.direction, q);

    if (v < 0.0f || u + v > 1.0f) return hit;

    // compute t to find out where the intersection point is on the line
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

    vector< Triangle* > triangles = meshes;  // copy for building
    bvhRoot = buildBVHNode(triangles, 0, triangles.size(), 0);
    cout << "BVH built for figure with " << meshes.size() << " triangles"
         << endl;
  }

  void addMesh(Triangle* tr) {
    tr->setMaterial(this->material);
    meshes.push_back(tr);
  }

  int getMode() { return mode; }

  Hit intersect(Ray ray) {
    Hit closest_hit;
    closest_hit.distance = INFINITY;
    closest_hit.hit = false;

    glm::vec3 localOrigin = glm::vec3(inverseTransformationMatrix * glm::vec4(ray.origin, 1.0f));
    glm::vec3 localDirection = glm::vec3(inverseTransformationMatrix * glm::vec4(ray.direction, 0.0f));
    localDirection = glm::normalize(localDirection);
    Ray localRay(localOrigin, localDirection);

    // use BVH if available, otherwise fall back to brute force
    if (bvhRoot) {
      closest_hit = intersectBVH(bvhRoot, localRay);
    } else {
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

BVHNode* buildBVHNode(vector<Triangle*>& triangles, int start, int end, int depth) {
    BVHNode* node = new BVHNode();
    
    for (int i = start; i < end; i++) {
        node->bbox.expand(computeTriangleBBox(triangles[i]));
    }
    
    int count = end - start;
    if (count <= 6 || depth >= 18) {
        node->triangles.assign(triangles.begin() + start, triangles.begin() + end);
        return node;
    }
    
    glm::vec3 extent = node->bbox.max - node->bbox.min;
    int axis = (extent.x > extent.y && extent.x > extent.z) ? 0 : (extent.y > extent.z) ? 1 : 2;
    
    auto comparator = [this, axis](Triangle* a, Triangle* b) {
        return computeTriangleBBox(a).getCenter()[axis] < 
               computeTriangleBBox(b).getCenter()[axis];
    };
    
    int mid = start + count / 2;
    std::nth_element(triangles.begin() + start, triangles.begin() + mid, 
                     triangles.begin() + end, comparator);
    
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
      // check all triangles in leaf node
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

glm::vec3 PhongModel(const Hit &hit, const glm::vec3 &view_direction) {
  glm::vec3 color(0.0);

  for (int light_num = 0; light_num < lights.size(); light_num++) {
    glm::vec3 light_direction = glm::normalize(lights[light_num]->position - hit.intersection);

    bool in_shadow = false;
    Ray shadowRay(hit.intersection + (100.0f * TOLERANCE * hit.normal), light_direction);
    for (int obj_num = 0; obj_num < objects.size(); ++obj_num) {
      Hit shadow_hit = objects[obj_num]->intersect(shadowRay);
      if (shadow_hit.hit && shadow_hit.distance < glm::distance(lights[light_num]->position, hit.intersection)) {
        in_shadow = true;
        break;
      }
    }

    if (in_shadow) continue;
    float NdotL = glm::clamp(glm::dot(hit.normal, light_direction), 0.0f, 1.0f);

    glm::vec3 diffuse_color = hit.object->getMaterial().diffuse;
    glm::vec3 diffuse = diffuse_color * glm::vec3(NdotL);
    color += lights[light_num]->color * diffuse;
  }
  color += ambient_light * hit.object->getMaterial().ambient;
  color = glm::clamp(color, glm::vec3(0.0), glm::vec3(1.0));
  return color;
}

glm::vec3 trace_ray(const Ray &ray) {
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
    color = PhongModel(closest_hit, glm::normalize(-ray.direction));
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
        // format: v/t/n - replace all slashes with spaces
        replace(processed.begin(), processed.end(), '/', ' ');
        break;
      }
      case WITH_NORMALS: {
        // format: v//n - replace double slash with space
        size_t pos = processed.find("//");
        if (pos != string::npos) {
          processed.replace(pos, 2, " ");
        }
        break;
      }
      default:
        // format: v - no processing needed
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

void loadMesh(Figure* figure, string filename) {
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
		} else if (token == "f") {
      processFace(figure, iss, normals, vertices, figure->getMaterial());
      ++faces;
		}
	}

	cout << "Finished loading mesh from: " << filename << endl;
	cout << "Vertices: " << vertices.size() << "; ";
	cout << "Normals: " << normals.size() << "; ";
	cout << "Polygons: " << faces << endl;
	file.close();
}

void sceneDefinition() {
  Material mirror;
  mirror.diffuse = glm::vec3(0.7);
  // mirror.ambient = glm::vec3(0.7);

  Figure* bunny = new Figure(mirror, WITHOUT_NORMALS);
  loadMesh(bunny, string("meshes/bunny.obj"));
  // loadMesh(bunny, string("meshes/bunny_small.obj"));
  bunny->buildBVH();
  glm::mat4 translationMatrixBunny = glm::translate(glm::vec3(0.0, -3.0, 8.0));
  bunny->setTransformation(translationMatrixBunny);
  objects.push_back(bunny);

  Figure* armadillo = new Figure(mirror, WITHOUT_NORMALS);
  loadMesh(armadillo, string("meshes/armadillo.obj"));
  // loadMesh(armadillo, string("meshes/armadillo_small.obj"));
  armadillo->buildBVH();
  glm::mat4 translationMatrixArma = glm::translate(glm::vec3(-4, -3.0, 10));
  armadillo->setTransformation(translationMatrixArma);
  objects.push_back(armadillo);

  Figure* lucy = new Figure(mirror, WITHOUT_NORMALS);
  loadMesh(lucy, string("meshes/lucy.obj"));
  // loadMesh(lucy, string("meshes/lucy_small.obj"));
  lucy->buildBVH();
  glm::mat4 translationMatrixLucy = glm::translate(glm::vec3(4.0, -3.0, 10));
  lucy->setTransformation(translationMatrixLucy);
  objects.push_back(lucy);

  lights.push_back(new Light(glm::vec3(0, 26, 5), glm::vec3(0.2)));
  lights.push_back(new Light(glm::vec3(0, 1, 12), glm::vec3(0.3)));
  lights.push_back(new Light(glm::vec3(0, 5, 1), glm::vec3(0.3)));

  objects.push_back(new Plane(glm::vec3(0, -3, 0), glm::vec3(0.0, 1, 0)));
  objects.push_back(new Plane(glm::vec3(0, 1, 30), glm::vec3(0.0, 0.0, -1.0)));
  objects.push_back(new Plane(glm::vec3(-15, 1, 0), glm::vec3(1.0, 0.0, 0.0)));
  objects.push_back(new Plane(glm::vec3(15, 1, 0), glm::vec3(-1.0, 0.0, 0.0)));
}

glm::vec3 toneMapping(glm::vec3 intensity) {
  float gamma = 1.0 / 2.0;
  float alpha = 12.0f;
  return glm::clamp(alpha * glm::pow(intensity, glm::vec3(gamma)), glm::vec3(0.0), glm::vec3(1.0));
}

atomic<int> pixels_rendered(0);

void renderTile(int startX, int endX, int startY, int endY, Image& image, float s, float X, float Y) {
    float dx = X + s/2;
    for (int i = startX; i < endX; i++) {
      float dy = Y - s/2;
      for (int j = startY; j < endY; j++) {
        Ray ray(dx, dy, 1);
        // image.setPixel(i, j, toneMapping(trace_ray(ray)));
        image.setPixel(i, j, trace_ray(ray));

        pixels_rendered++;
        dy -= s;
      }
      dx += s;
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
    ios_base::sync_with_stdio(0);
    cin.tie(0);
    // auto t0 = chrono::high_resolution_clock::now();
    clock_t t = clock();  // variable for keeping the time of the rendering
    int width = 2048;
    int height = 1536;
    float fov = 90;

    sceneDefinition();

    Image image(width, height);
    
    float s = 2 * tan(0.5 * fov / 180 * M_PI) / width;
    float X = -s * width / 2;
    float Y = s * height / 2;

    // progress thread
    thread progressThread(printProgress, width * height);

    renderTile(0, width, 0, height, image, s, X, Y);

    progressThread.join();

    // auto t1 = chrono::high_resolution_clock::now();
    // auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(t1 - t0);
    // float seconds = duration.count() / 1000.0f;
    // float fps = 1.0f / seconds;

    // std::cout << "It took " << seconds << " seconds to render the image." << std::endl;
    // std::cout << "I could render at " << fps << " frames per second." << std::endl;

    t = clock() - t;
    cout << "It took " << ((float)t) / CLOCKS_PER_SEC
         << " seconds to render the image." << endl;
    cout << "I could render at " << (float)CLOCKS_PER_SEC / ((float)t)
         << " frames per second." << endl;
    if (argc == 2) {
        image.writeImage(argv[1]);
    } else {
        image.writeImage("./result.ppm");
    }

    // cleanup
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