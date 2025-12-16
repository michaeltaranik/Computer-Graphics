//
//  Material.h
//  Raytracer
//
//  Created by Piotr Didyk on 14.07.21.
//

#ifndef Material_h
#define Material_h

#include "glm/glm.hpp"

/**
 Structure describing a material of an object
 */
enum MaterialType { PHONG, WARD, EMISSIVE };

struct Material {
  glm::vec3 ambient;
  glm::vec3 diffuse;
  glm::vec3 specular;
  float shininess;

  float kreflect = 0.0f;
  float krefract = 0.0f;
  float refractIdx = 1.0f;
  float transparency = 0.0f;

  MaterialType type = PHONG;
  float alphaX = 0.1f;  // Roughness in X (Tangent direction)
  float alphaY = 0.1f;  // Roughness in Y (Bitangent direction)
};

#endif /* Material_h */
