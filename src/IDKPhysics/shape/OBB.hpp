#pragma once
#include "shape.hpp"


class idk::phys::ShapeOBB: public Shape
{
public:
    ShapeOBB( const glm::vec3 &extents );
    ShapeOBB( float w, float h, float d );

    virtual bool  raycast( const glm::vec3&, const glm::vec3&, glm::vec3*, glm::vec3* );
    virtual float getArea( const glm::vec3& );
    virtual bool  collides( Shape*, CollisionInfo *info=nullptr );
};


