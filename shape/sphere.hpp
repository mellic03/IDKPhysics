#pragma once
#include "shape.hpp"


namespace idk::phys
{
    bool shapes_intersect( ShapeSphere *A, ShapeSphere *B );
}


class idk::phys::ShapeSphere: public Shape
{
public:
    ShapeSphere(): Shape(Type::SPHERE) {  };
    virtual bool intersects( Shape*) final;

};
