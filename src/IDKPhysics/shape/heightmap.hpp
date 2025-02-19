#pragma once
#include "shape.hpp"

namespace idk::phys
{
    class ShapeHeightmap;
}


class idk::phys::ShapeHeightmap: public Shape
{
public:
    // ShapeHeightmap( uint32_t w, uint32_t h, void *pixels );
    ShapeHeightmap();

    virtual bool  raycast( const glm::vec3&, const glm::vec3&,
                           glm::vec3*, glm::vec3* ) override;

    virtual float getArea( const glm::vec3& );

    // virtual bool  collides( Shape*, CollisionInfo *info=nullptr );

};
