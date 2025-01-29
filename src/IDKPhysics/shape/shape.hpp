#pragma once
#include "../common.hpp"
#include <IDKPhysics/body/state.hpp>

namespace idk::phys
{
    class Body;
}


// #define IDKPhysShape_VERYLAZY(typeA, typeB)\
// template<> bool Shape##typeA::_collides<Shape##typeB>( Shape *shape, CollisionInfo *info )\
// {\
//     return idk::phys::collision##typeA##typeB(*this, *dynamic_cast<Shape##typeB*>(shape), info);\
// }\

#define SHAPE_COLLISION_LAZY(typeA, typeB)\
if (dynamic_cast<Shape##typeB*>(other))\
return idk::phys::collisionTest(*this, *dynamic_cast<Shape##typeB*>(other), info);\


class idk::phys::Shape
{
protected:
    friend class Body;

public:
    Body *body;
    glm::vec3 extents;

    Shape( const glm::vec3 &extents = glm::vec3(1.0f) )
    :   extents(extents)
    {
        
    }

    virtual bool  raycast( const glm::vec3&, const glm::vec3&, glm::vec3*, glm::vec3* ) = 0;
    virtual float getArea( const glm::vec3& ) = 0;
    virtual bool  collides( Shape*, CollisionInfo* ) = 0;

};

