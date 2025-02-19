#include "state.hpp"
#include "body.hpp"

using namespace idk::phys;


KinematicForce::KinematicForce()
:   dAcc(0), dVel(0), dPos(0), acc(0), vel(0)
{

}

KinematicState::KinematicState( const glm::vec3 &p, const glm::quat &r )
:   pos         (p),
    rot         (r),
    COM_local   (0.0f),
    COM_world   (p),
    invMass     (1.0/1.0),
    drag        (0.05),
    restitution (0.66),
    up          (0, 1, 0),
    right       (1, 0, 0),
    front       (0, 0, 1)
{

}



// KinematicState
// KinematicState::mix( const KinematicState &A, const KinematicState &B, float alpha )
// {
//     KinematicState AB(glm::vec3(0.0f));

//     AB.pos = glm::mix(A.pos, B.pos, alpha);
//     // AB.COM = glm::mix(A.COM, B.COM, alpha);
//     AB.rot = glm::slerp(A.rot, B.rot, alpha);
//     AB.T   = glm::translate(glm::mat4(1), AB.pos) * glm::mat4_cast(AB.rot);

//     return AB;
// }




void 
KinematicState::applyTranslation( const glm::vec3 &v )
{
    linear.dPos += v;
}


void
KinematicState::applyImpulse( const glm::vec3 &Jworld, const glm::vec3 &Pworld )
{
    applyImpulseLinear(Jworld);
    applyImpulseAngular(Jworld, Pworld);
}


void 
KinematicState::applyImpulseLinear( const glm::vec3 &Jworld )
{
    linear.dVel += Jworld;
}

void
KinematicState::applyImpulseAngular( const glm::vec3 &worldAxis )
{
    angular.dVel += worldAxis;
}

void 
KinematicState::applyImpulseAngular( const glm::vec3 &Jworld, const glm::vec3 &Pworld )
{
    angular.dVel += computeTorque(Jworld, Pworld);
}



void
KinematicState::applyForce( const glm::vec3 &Fworld, const glm::vec3 &Pworld )
{
    applyForceLinear(Fworld);
    applyForceAngular(Fworld, Pworld);
}

void
KinematicState::applyForceLinear( const glm::vec3 &Fworld )
{
    linear.dAcc += Fworld;
}

void
KinematicState::applyForceAngular( const glm::vec3 &worldAxis )
{
    angular.dAcc += worldAxis;
}

void
KinematicState::applyForceAngular( const glm::vec3 &Fworld, const glm::vec3 &Pworld )
{
    angular.dAcc += computeTorque(Fworld, Pworld);
}



glm::vec3
KinematicState::computeTorque( const glm::vec3 &Fworld, const glm::vec3 &Pworld )
{
    glm::vec3 r_world = Pworld - COM_world;

    if (glm::length2(r_world) < 0.0001f || glm::length2(Fworld) < 0.0001f)
    {
        return glm::vec3(0.0f);
    }

    return glm::cross(r_world, Fworld);
}


void
KinematicState::updateTransforms()
{
    T     = glm::translate(glm::mat4(1), pos) * glm::mat4_cast(rot);
    Ti    = glm::inverse(T);
    up    = T * glm::vec4(0, 1, 0, 0);
    right = T * glm::vec4(1, 0, 0, 0);
    front = T * glm::vec4(0, 0, 1, 0);

    COM_local = glm::vec3(0.0f);
    COM_world = T * glm::vec4(COM_local, 1.0f);

    I   = glm::mat3(1.0f);
    Iwi = glm::mat3(T) * I * glm::mat3(Ti);
    Iwi = glm::inverse(Iwi);
}


void
KinematicState::clearForces()
{
    linear.dAcc  *= 0.0f;
    linear.dVel *= 0.0f;
    linear.dPos *= 0.0f;

    angular.dAcc *= 0.0f;
    angular.dVel *= 0.0f;
}

