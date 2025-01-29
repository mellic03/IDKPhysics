#include "AABB.hpp"
#include "OBB.hpp"
#include "sphere.hpp"
#include "heightmap.hpp"
#include "collision.hpp"
#include <IDKPhysics/body/body.hpp>
#include "../query.hpp"

using namespace idk::phys;


ShapeAABB::ShapeAABB( const glm::vec3 &extents )
:   Shape(extents) {  }

ShapeAABB::ShapeAABB( float w, float h, float d )
:   ShapeAABB(glm::vec3(w, h, d)) {  }


bool
ShapeAABB::raycast( const glm::vec3 &ro, const glm::vec3 &rd, glm::vec3 *hit, glm::vec3 *N )
{
    return false;
}


float
ShapeAABB::getArea( const glm::vec3 &dir )
{
    return extents.x;
}


bool
ShapeAABB::collides( Shape *other, CollisionInfo *info )
{
    SHAPE_COLLISION_LAZY(AABB, AABB);
    SHAPE_COLLISION_LAZY(AABB, Heightmap);
    SHAPE_COLLISION_LAZY(AABB, OBB);
    SHAPE_COLLISION_LAZY(AABB, Sphere);
    return false;
}


// template <>
// bool
// ShapeAABB::_collides<ShapeAABB>( Shape *shape, CollisionInfo *info )
// {
//     return collisionCuboidCuboid(*this, *dynamic_cast<ShapeAABB*>(shape), info);
// }


// template <>
// bool
// ShapeAABB::_collides<ShapeSphere>( Shape *shape, CollisionInfo *info )
// {
//     return collisionCuboidSphere(*this, *dynamic_cast<ShapeSphere*>(shape), info);
// }



