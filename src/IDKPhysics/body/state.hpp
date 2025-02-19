#pragma once
#include "../common.hpp"


namespace idk::phys
{
    struct KinematicForce;
    struct KinematicState;
}

struct idk::phys::KinematicForce
{
    glm::vec3 dAcc, dVel, dPos;
    glm::vec3 acc, vel;
    KinematicForce();
};


struct idk::phys::KinematicState
{
    float movementscale = 1.0f;

    float invMass, drag, restitution;

    KinematicForce linear, angular;

    glm::vec3    COM_world;
    glm::vec3    COM_local;
    glm::vec3    pos;
    glm::quat    rot;
    glm::mat4    T   = glm::mat4(1.0f);
    glm::mat4    Ti  = glm::mat4(1.0f);

    /** Inertia, world, inverse */
    glm::mat3    I   = glm::mat3(1.0f);
    glm::mat3    Iwi = glm::mat3(1.0f);

    glm::vec3 up, right, front;

    KinematicState( const glm::vec3 &p=glm::vec3(0.0f), const glm::quat &r=glm::quat(glm::vec3(0.0f)) );

    float getMass() { return 1.0f/invMass; }
    void  setMass( float m ) { invMass = 1.0f/m; }

    void  applyTranslation( const glm::vec3& );

    void  applyImpulse( const glm::vec3 &worldVel, const glm::vec3 &worldPoint );
    void  applyImpulseLinear( const glm::vec3 &worldVel );
    void  applyImpulseAngular( const glm::vec3 &worldAxis );
    void  applyImpulseAngular( const glm::vec3 &worldVel, const glm::vec3 &worldPoint );

    void  applyForce( const glm::vec3 &worldAcc, const glm::vec3 &worldPoint );
    void  applyForceLinear( const glm::vec3 &worldAcc );
    void  applyForceAngular( const glm::vec3 &worldAxis );
    void  applyForceAngular( const glm::vec3 &worldAcc, const glm::vec3 &worldPoint );

    glm::vec3 computeTorque( const glm::vec3 &worldAcc, const glm::vec3 &worldPoint );

    void  updateTransforms();
    void  clearForces();

};

