#include "sphere.hpp"


bool
idk::phys::shapes_intersect( ShapeSphere *A, ShapeSphere *B )
{
    return false;
}


bool
idk::phys::ShapeSphere::intersects( Shape *other )
{
    switch (other->getType())
    {
        default: return false;
        case Type::SPHERE: return shapes_intersect(this, this); break;
        case Type::CUBOID: return shapes_intersect(this, this); break;
    }
}

