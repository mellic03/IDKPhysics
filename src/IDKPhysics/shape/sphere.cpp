#include "sphere.hpp"
#include "AABB.hpp"
#include "OBB.hpp"
#include "heightmap.hpp"
#include "collision.hpp"
#include <IDKPhysics/body/body.hpp>
#include <libidk/idk_math.hpp>
#include "../query.hpp"

using namespace idk::phys;


ShapeSphere::ShapeSphere( float r )
:   Shape(glm::vec3(r, r, r)),
    radius(extents[0])
{

}


bool
ShapeSphere::raycast( const glm::vec3 &ro, const glm::vec3 &rd, glm::vec3 *hit, glm::vec3 *N )
{
    return raySphereIntersect(ro, rd, body->state.pos, extents.x, hit, N);
}


float
ShapeSphere::getArea( const glm::vec3 &dir )
{
    float r = extents.x;
    return idk::PI * r*r;
}


bool
ShapeSphere::collides( Shape *other, CollisionInfo *info )
{
    SHAPE_COLLISION_LAZY(Sphere, AABB);
    SHAPE_COLLISION_LAZY(Sphere, OBB);
    SHAPE_COLLISION_LAZY(Sphere, Heightmap);
    SHAPE_COLLISION_LAZY(Sphere, Sphere);
    return false;
}


// template <>
// bool
// ShapeSphere::_collides<ShapeCuboid>( Shape *shape, CollisionInfo *info )
// {
//     return idk::phys::collisionCuboidSphere(*dynamic_cast<ShapeCuboid*>(shape), *this, info);
// }


// template <>
// bool
// ShapeSphere::_collides<ShapeSphere>( Shape *shape, CollisionInfo *info )
// {
//     return idk::phys::collisionSphereSphere(*this, *dynamic_cast<ShapeSphere*>(shape), info);
// }



