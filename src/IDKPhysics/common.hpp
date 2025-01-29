#pragma once

#include <libidk/idk_glm.hpp>


namespace idk::phys
{
    class BodyState;
    class Body;

    class StaticBody;
    class RigidBody;
    class SoftBody;

    class Shape;
    class ShapeSphere;
    class ShapeAABB;
    class ShapeOBB;
    class ShapeHeightmap;

    class WorldConfig;
    class World;

    glm::vec3 computeTorqueAngularForce( const glm::vec3 &offset, const glm::vec3 &F );

    // using vec2type = glm::vec2;
    // using vec3type = glm::vec3;
    // using vec4type = glm::vec4;
    // using mat3type = glm::mat3;
    // using mat4type = glm::mat4;

    // using glm::vec2 = glm::highp_f64vec2;
    // using glm::vec3 = glm::highp_f64vec3;
    // using glm::vec4 = glm::highp_f64vec4;
    // using glm::mat3 = glm::highp_f64mat4;
    // using glm::mat4 = glm::highp_f64mat4;

}

