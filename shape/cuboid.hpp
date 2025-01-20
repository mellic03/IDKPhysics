#pragma once
#include "shape.hpp"


class idk::phys::ShapeCuboid: public Shape
{
public:
    ShapeCuboid(): Shape(Type::CUBOID) {  };
    virtual bool intersects( Shape*) final;

};


