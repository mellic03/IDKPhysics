#include "common.hpp"
#include "query.hpp"



bool
idk::phys::raySphereIntersect( const glm::vec3 &ro, const glm::vec3 &rd, 
                               const glm::vec3 &so, float r,
                               glm::vec3 *hit, glm::vec3 *normal )
{
    glm::vec3 L = so - ro;

    float l   = glm::length(L);
    float tc  = glm::dot(rd, L);
    float d   = glm::sqrt(l*l - tc*tc);
    float t1c = glm::sqrt(r*r - d*d);

    if (d >= r)
    {
        return false;
    }

    float t1 = tc - t1c;
    float t2 = tc + t1c;

    if (hit)
    {
        if (t1 < 0.0f)
        {
            *hit = ro + t2*rd;
        }

        else
        {
            *hit = ro + t1*rd;
        }

        if (normal)
        {
            *normal = glm::normalize(*hit - so);
        }
    }

    return true;
}
