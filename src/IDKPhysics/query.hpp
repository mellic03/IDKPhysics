#pragma once
#include <libidk/idk_glm.hpp>

namespace idk::phys
{
    bool raySphereIntersect( const glm::vec3 &ro, const glm::vec3 &rd, 
                             const glm::vec3 &so, float r,
                             glm::vec3 *hit, glm::vec3 *normal );
}
