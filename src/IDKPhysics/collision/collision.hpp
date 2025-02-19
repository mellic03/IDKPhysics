#pragma once
#include "collision-info.hpp"
#include "collision-list.hpp"
#include <IDKPhysics/shape/shape.hpp>


namespace idk::phys
{
    void registerMethods();
    bool detectCollisionShapeShape( Shape*, Shape*, CollisionInfo& );
    bool resolveCollisionShapeShape( CollisionInfo& );
}


#undef GEN_REVERSE_FUNCTION