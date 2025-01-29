#include "common.hpp"


glm::vec3
idk::phys::computeTorqueAngularForce( const glm::vec3 &offset, const glm::vec3 &F )
{
    glm::vec3 r = offset;
    return glm::cross(r, F);
}
