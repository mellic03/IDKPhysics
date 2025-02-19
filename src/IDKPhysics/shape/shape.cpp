#include "shape.hpp"


idk::phys::Shape
idk::phys::ShapeAABB( const glm::vec3 &extents )
{
    return Shape(SHAPE_AABB, extents);
}

idk::phys::Shape
idk::phys::ShapeAABB( float w, float h, float d )
{
    return Shape(SHAPE_AABB, glm::vec3(w, h, d));
}


idk::phys::Shape
idk::phys::ShapeOBB( const glm::vec3 &extents )
{
    return Shape(SHAPE_OBB, extents);
}

idk::phys::Shape
idk::phys::ShapeOBB( float w, float h, float d )
{
    return Shape(SHAPE_OBB, glm::vec3(w, h, d));
}




bool
idk::phys::Shape::raycast( const glm::vec3&, const glm::vec3&, glm::vec3*, glm::vec3* )
{
    return false;
}


float
idk::phys::Shape::getArea( const glm::vec3 &dir )
{
    return 1.0f;
}

