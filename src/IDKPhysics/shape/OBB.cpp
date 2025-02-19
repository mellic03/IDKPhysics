// #include "OBB.hpp"
// #include <IDKPhysics/body/body.hpp>
// #include "../query.hpp"

// using namespace idk::phys;


// ShapeOBB::ShapeOBB( const glm::vec3 &extents )
// :   Shape(SHAPE_OBB, extents) {  }

// ShapeOBB::ShapeOBB( float w, float h, float d )
// :   ShapeOBB(glm::vec3(w, h, d)) {  }


// bool
// ShapeOBB::raycast( const glm::vec3 &ro, const glm::vec3 &rd, glm::vec3 *hit, glm::vec3 *N )
// {
//     return false;
// }


// float
// ShapeOBB::getArea( const glm::vec3 &dir )
// {
//     return extents.x;
// }


// // bool
// // ShapeOBB::collides( Shape *other, CollisionInfo *info )
// // {
// //     // SHAPE_COLLISION_LAZY(AABB, OBB);
// //     // SHAPE_COLLISION_LAZY(OBB, OBB);
// //     // SHAPE_COLLISION_LAZY(OBB, Heightmap);
// //     // SHAPE_COLLISION_LAZY(OBB, Sphere);
// //     return false;
// // }


// // template <>
// // bool
// // ShapeOBB::_collides<ShapeOBB>( Shape *shape, CollisionInfo *info )
// // {
// //     return collisionCuboidCuboid(*this, *dynamic_cast<ShapeOBB*>(shape), info);
// // }


// // template <>
// // bool
// // ShapeOBB::_collides<ShapeSphere>( Shape *shape, CollisionInfo *info )
// // {
// //     return collisionCuboidSphere(*this, *dynamic_cast<ShapeSphere*>(shape), info);
// // }



