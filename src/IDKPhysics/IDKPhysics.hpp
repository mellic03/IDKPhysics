#pragma once

#include "body/body.hpp"
#include "body/staticbody.hpp"
#include "body/rigidbody.hpp"
#include "body/softbody.hpp"
#include "body/verletbody.hpp"
#include "shape/sphere.hpp"
#include "shape/AABB.hpp"
#include "shape/OBB.hpp"
#include "shape/heightmap.hpp"
#include "constraint/spring.hpp"
#include "constraint/distance.hpp"
#include "constraint/glue.hpp"
#include "constraint/verlet.hpp"
#include "world.hpp"

#include <optional>


namespace idk::phys
{
    float approxAirDensity( float altitude, float temperature );
    float approxAirDensity( float altitude );
    // void createRope( World &world );
    void createRope( World &world, RigidBody *root );

}



