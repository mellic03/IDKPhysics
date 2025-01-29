#pragma once
#include "shape.hpp"


class idk::phys::ShapeAABB: public Shape
{
public:
    ShapeAABB( const glm::vec3 &extents );
    ShapeAABB( float w, float h, float d );

    virtual bool  raycast( const glm::vec3&, const glm::vec3&, glm::vec3*, glm::vec3* );
    virtual float getArea( const glm::vec3& );
    virtual bool  collides( Shape*, CollisionInfo *info=nullptr );
};


