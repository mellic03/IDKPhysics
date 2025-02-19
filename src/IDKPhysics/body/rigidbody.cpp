#include "rigidbody.hpp"
#include "staticbody.hpp"
#include "../world.hpp"
#include <IDKPhysics/shape/sphere.hpp>
#include <IDKPhysics/shape/OBB.hpp>
#include <libidk/idk_math.hpp>
#include <IDKGraphics/terrain/terrain.hpp>

using namespace idk::phys;


RigidBody::RigidBody( World &world, KinematicState &st, const Shape &S )
:   Body(world, st, S)
{
    // _onChange();

    // onCollisionEnter = [this](const phys::CollisionInfo &info)
    // {
    //     _defaultOnCollisionEnter(info);
    // };
}


RigidBody::RigidBody( World &world, KinematicState &st, shape_type stype )
:   RigidBody(world, st, Shape(stype))
{

}




void
RigidBody::update( float alpha )
{
    Body::update(alpha);
    // float radius = std::pow(1.0f/state.invMass / (4.0f/3.0f * idk::PI), 1.0f/3.0f);
    // shape->extents = glm::vec3(0.5f * radius);
}


void
RigidBody::integrate( float dt )
{
    auto  &conf = m_world.config;
    float factor = 1.0f / conf.substeps;

    applyForceLinear(computeAeroForcesLinear());
    applyForceAngular(computeAeroForcesAngular());


    // glm::vec3 aero = computeAeroForcesAngular();

    // updateTransforms();
    // float totalMass = _computeTotalMass();
    // state.COM   = _computeCOM(totalMass);
    // updateTransforms();

    // state.I    = _computeIntertiaTensor(state.COM);
    // state.Iwi  = glm::mat3(state.T) * state.I * glm::mat3(state.Ti);
    // state.Iwi = glm::inverse(state.Iwi);
    // computeGravitationalAcceleration();

    // _integrateLinear(dt, factor);
    // _integrateAngular(dt, factor);
    // updateTransforms();
    // clearForces();
}



// void
// RigidBody::_integrateLinear( float dt, float factor )
// {
//     auto &acc = state.linear.acc;
//     auto &vel = state.linear.vel;
//     auto &pos = state.pos;

//     auto F = state.linear.dAcc + computeAeroForcesLinear();

//     acc  = invMass * F;
//     vel += dt*acc + invMass * (state.linear.dVel);
//     pos += dt*vel;
// }


// void
// RigidBody::_integrateAngular( float dt, float factor )
// {
//     static constexpr float damping = 0.025f;

//     auto &acc = state.angular.acc;
//     auto &vel = state.angular.vel;
//     auto &rot = state.rot;

//     acc  = Inertia.inverseWorld * state.angular.dAcc; // + G;
//     vel += dt*acc + Inertia.inverseWorld * state.angular.dVel;
//     vel *= (1.0f - dt*damping);

//     float magSq = glm::length2(state.angular.vel);
//     if (magSq < 1e-12f)
//     {
//         vel = glm::vec3(0.0f);
//         return;
//     }

//     float mag = glm::sqrt(magSq);
//     auto axis = vel / mag;
//     rot = glm::angleAxis(dt*mag, axis) * rot;
//     rot = glm::normalize(rot);
// }




// float
// RigidBody::_computeTotalMass( KinematicState &st )
// {
//     float totalMass = 1.0f/state.invMass;

//     for (auto *B: m_children)
//     {
//         totalMass += B->getMass();
//     }

//     return totalMass;
// }


// glm::vec3
// RigidBody::_computeCOM( float totalMass )
// {
//     auto COM = glm::vec3(0.0f);

//     for (auto *B: m_children)
//     {
//         COM += B->getMass() * B->state.pos;
//     }

//     COM += 1.0f/state.invMass * state.pos;

//     return COM / totalMass;
// }


// glm::mat3
// RigidBody::_computeIntertiaTensor( const glm::vec3 &COM )
// {
//     auto I = glm::mat3(1.0f);

//     glm::vec3 r;
//     float     m;
    
//     r  = state.pos - COM;
//     m  = getMass();
//     I += m * (glm::length2(r) * glm::mat3(1.0f) - glm::outerProduct(r, r));

//     for (auto *B: m_children)
//     {
//         r  = B->state.pos - COM;
//         m  = B->getMass();
//         I += m * (glm::length2(r) * glm::mat3(1.0f) - glm::outerProduct(r, r));
//     }

//     return I;
// }


std::pair<glm::vec3, glm::vec3>
RigidBody::computeGravitationalAcceleration()
{
    auto &conf = m_world.config;

    glm::vec3 G   = conf.substepFactor() * conf.gravity;
    glm::vec3 lin = glm::vec3(0.0f);
    glm::vec3 ang = glm::vec3(0.0f);

    // const auto do_thing = [this, G, &lin, &ang]( RigidBody *B )
    // {
    //     float factor = B->getMass() * state.invMass;
    //     applyImpulseLinear(factor*G);
    //     applyImpulseAngular(factor*G, B->state.COM);
    // };

    // do_thing(this);

    // for (auto *B: m_children)
    // {
    //     do_thing(B);
    // }

    return std::make_pair(lin, ang);
}


glm::vec3
RigidBody::computeAeroForcesLinear()
{
    auto &conf = m_world.config;
    float area = shape.getArea(state.linear.vel);
    auto  drag = -conf.fluid_density * state.drag * area * state.linear.vel;

    if (glm::length2(state.linear.vel) > 0.0f)
    {
        drag *= glm::length(state.linear.vel);
    }

    return drag;
}


glm::vec3
RigidBody::computeAeroForcesAngular()
{
    auto &conf = m_world.config;
    auto  drag = -conf.fluid_density * state.drag * state.angular.vel;

    if (glm::length2(state.angular.vel) > 0.0f)
    {
        drag *= glm::length(state.angular.vel);
    }
    
    return drag;
}




static glm::vec3 Jtemp;

// void
// RigidBody::addChild( RigidBody *B )
// {
//     B->m_parent = this;
//     m_children.push_back(B);

//     B->onCollisionEnter = [B](const phys::CollisionInfo &info)
//     {
//         B->_defaultOnCollisionEnter(info);
//         // B->m_parent->applyImpulse(B->state.pos, Jtemp);
//     };

//     _onChange();
// }



// void 
// RigidBody::_defaultOnCollisionEnter( const CollisionInfo &info )
// {
//     // if (dynamic_cast<StaticBody*>(info.B.body) == nullptr)
//     // {
//     //     return;
//     // }

//     // auto &p = info.contactPoint;
//     // auto &N = info.A.normal;
//     // auto *A = dynamic_cast<RigidBody*>(info.A.body);
//     // auto &stateA = A->state;

//     // auto *B = info.B.body;
//     // auto &stateB = B->state;

//     // float m1 = 1.0f / stateA.invMass;
//     // float m2 = 1.0f / stateB.invMass;
//     // float e1 = 0.98f; // stateA.restitution;
//     // float e2 = 0.98f; // stateB.restitution;

//     // glm::vec3 Av  = stateA.linear.vel;
//     // glm::vec3 Bv  = stateB.linear.vel;
//     // glm::vec3 Vr  = Av - Bv;
//     // float     vj  = -(1.0f + e1) * glm::dot(Vr, N) * (m1 + m2);
//     // glm::vec3 J   = vj * N;

//     // Jtemp = J;

//     // stateA.applyImpulseLinear(J/m1);
//     // stateA.applyTranslation(-info.A.penetration);

// }
