#pragma once
#include "../common.hpp"
#include <IDKPhysics/body/state.hpp>


namespace idk::phys
{
    enum shape_type: uint32_t
    {
        SHAPE_NONE      = 0,
        SHAPE_AABB      = 1<<1,
        SHAPE_HEIGHTMAP = 1<<2,
        SHAPE_OBB       = 1<<3,
        SHAPE_SPHERE    = 1<<4,
    };

    phys::Shape ShapeAABB( const glm::vec3& );
    phys::Shape ShapeAABB( float w, float h, float d );

    phys::Shape ShapeOBB( const glm::vec3& );
    phys::Shape ShapeOBB( float w, float h, float d );
}


class idk::phys::Shape
{
protected:
    friend class Body;

public:
    const shape_type type;
    glm::vec3  extents;
    float      radius;
    Body      *body;

    Shape( shape_type stype = SHAPE_NONE, const glm::vec3 &e = glm::vec3(1.0f) )
    :   type(stype), extents(e), radius(1.0f)
    {
        
    }

    virtual bool  raycast( const glm::vec3&, const glm::vec3&, glm::vec3*, glm::vec3* );
    virtual float getArea( const glm::vec3& );
    // virtual bool  collides( Shape*, CollisionInfo* ) = 0;

};

