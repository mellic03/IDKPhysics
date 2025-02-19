#pragma once
#include "../common.hpp"
#include "body.hpp"


class idk::phys::StaticBody: public idk::phys::Body
{
private:
    friend class World;

public:
    StaticBody( idk::phys::World&, KinematicState&, phys::shape_type stype = SHAPE_SPHERE );
    StaticBody( idk::phys::World&, KinematicState&, const Shape& );
};

