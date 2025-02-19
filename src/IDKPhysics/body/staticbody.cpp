#include "staticbody.hpp"
#include "../world.hpp"



idk::phys::StaticBody::StaticBody( World &world, KinematicState &st, const Shape &S )
:   Body(world, st, S)
{
    state.movementscale = 0.0f;
    state.invMass = 0.0f;
    state.restitution = 0.15f;
}



idk::phys::StaticBody::StaticBody( World &world, KinematicState &st, shape_type stype )
:   StaticBody(world, st, phys::Shape(stype))
{
    
}

