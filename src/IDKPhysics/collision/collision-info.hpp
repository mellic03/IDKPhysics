#pragma once

#include <glm/glm.hpp>


namespace idk::phys
{
    struct Body;
    struct CollisionInfo;
}


struct idk::phys::CollisionInfo
{
public:
    Body *A, *B;

    /** Contact point. */
    glm::vec3 contact;

    /** Collision normal pointing from B to A. Adding to A will move it away from B. */
    glm::vec3 normal;

    float penetration = 1.0f;

};
