#pragma once
#include "shape.hpp"
#include <IDKPhysics/body/state.hpp>


#define GEN_REVERSE_FUNCTION(typeA, typeB)\
inline static bool collisionTest( Shape##typeA &A, Shape##typeB &B, CollisionInfo *info )\
{ return collisionTest(B, A, info); }\


namespace idk::phys
{
    bool collisionTest( ShapeAABB&, ShapeAABB&, CollisionInfo* );
    bool collisionTest( ShapeAABB&, ShapeHeightmap&, CollisionInfo* );
    bool collisionTest( ShapeAABB&, ShapeOBB&, CollisionInfo* );
    bool collisionTest( ShapeAABB&, ShapeSphere&, CollisionInfo* );
    GEN_REVERSE_FUNCTION(Heightmap, AABB)
    GEN_REVERSE_FUNCTION(OBB, AABB)
    GEN_REVERSE_FUNCTION(Sphere, AABB)

    bool collisionTest( ShapeHeightmap&, ShapeOBB&, CollisionInfo* );
    bool collisionTest( ShapeHeightmap&, ShapeSphere&, CollisionInfo* );
    GEN_REVERSE_FUNCTION(Sphere, Heightmap)
    GEN_REVERSE_FUNCTION(OBB, Heightmap)

    bool collisionTest( ShapeOBB&, ShapeOBB&, CollisionInfo* );
    bool collisionTest( ShapeOBB&, ShapeSphere&, CollisionInfo* );
    GEN_REVERSE_FUNCTION(Sphere, OBB)

    bool collisionTest( ShapeSphere&, ShapeSphere&, CollisionInfo* );
}


#undef GEN_REVERSE_FUNCTION