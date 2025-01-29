#include "heightmap.hpp"
#include "sphere.hpp"
#include "AABB.hpp"
#include "OBB.hpp"
#include "collision.hpp"
#include <IDKPhysics/body/body.hpp>
#include <IDKGraphics/terrain/terrain.hpp>
#include "../query.hpp"

using namespace idk::phys;


// ShapeHeightmap::ShapeHeightmap( uint32_t w, uint32_t h, void *pixels )
ShapeHeightmap::ShapeHeightmap()
{

}


bool
ShapeHeightmap::raycast( const glm::vec3 &ro, const glm::vec3 &rd, glm::vec3 *hit, glm::vec3 *N )
{
    return false;
}


float
ShapeHeightmap::getArea( const glm::vec3 &dir )
{
    return 1.0f;
}


bool
ShapeHeightmap::collides( Shape *other, CollisionInfo *info )
{
    SHAPE_COLLISION_LAZY(Heightmap, AABB);
    SHAPE_COLLISION_LAZY(Heightmap, OBB);
    SHAPE_COLLISION_LAZY(Heightmap, Sphere);
    return false;
}

