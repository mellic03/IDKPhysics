#pragma once
#include "shape.hpp"


class idk::phys::ShapeSphere: public Shape
{
public:
    float &radius;
    ShapeSphere( float radius = 1.0f );

    virtual bool  raycast( const glm::vec3&, const glm::vec3&, glm::vec3*, glm::vec3* );
    virtual float getArea( const glm::vec3& );
    virtual bool  collides( Shape*, CollisionInfo *info=nullptr );

};

