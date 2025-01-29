#include "rigidbody.hpp"
#include "staticbody.hpp"
#include "../world.hpp"
#include <IDKPhysics/shape/sphere.hpp>
#include <IDKPhysics/shape/OBB.hpp>
#include <libidk/idk_math.hpp>
#include <IDKGraphics/terrain/terrain.hpp>

using namespace idk::phys;


RigidBody::RigidBody( World &world, const glm::vec3 &pos, Shape *shape )
:   Body(world, pos, shape)
{
    _onChange();
    integrate(1.0f/60.0f);

    onCollisionEnter = [this](const phys::CollisionInfo &info)
    {
        return _defaultOnCollisionEnter(info);
    };
}



void
RigidBody::update( float alpha )
{
    Body::update(alpha);
    // float radius = std::pow(m_mass / (4.0f/3.0f * idk::PI), 1.0f/3.0f);
    // m_shape->extents = glm::vec3(0.5f * radius);
}




float
RigidBody::_computeTotalMass()
{
    float totalMass = m_mass;

    for (auto *B: m_children)
    {
        totalMass += B->getMass();
    }

    return totalMass;
}


glm::vec3
RigidBody::_computeCOM( float totalMass )
{
    auto COM = glm::vec3(0.0f);

    for (auto *B: m_children)
    {
        COM += B->getMass() * (B->state.pos - this->state.pos);
    }

    return COM / totalMass;
}


glm::mat3
RigidBody::_computeIntertiaTensor( const glm::vec3 &COM )
{
    auto I = glm::mat3(0.0f);

    for (auto *B: m_children)
    {
        glm::vec3 r = (B->state.pos - this->state.pos) - COM;
        float     m = B->getMass();

        I += m * (glm::length2(r) * glm::mat3(1.0f) - glm::outerProduct(r, r));
    }

    return I;
}


void
RigidBody::_onChange()
{
    m_totalMass = _computeTotalMass();
    invMass     = 1.0f / m_totalMass;
    m_COM       = _computeCOM(m_totalMass);

    updateTransforms();

    Inertia.local        = _computeIntertiaTensor(m_COM);
    Inertia.inverse      = glm::inverse(Inertia.local);
    Inertia.world        = state.T3 * Inertia.local * state.invT3;
    Inertia.inverseWorld = glm::inverse(Inertia.world);
}



glm::vec3
RigidBody::computeAeroForcesLinear()
{
    auto &conf = m_world.config;
    float area = m_shape->getArea(state.linear.vel);
    auto  drag = -conf.fluid_density * m_drag * area * state.linear.vel;

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
    auto  drag = -conf.fluid_density * m_drag * state.angular.vel;

    if (glm::length2(state.angular.vel) > 0.0f)
    {
        drag *= glm::length(state.angular.vel);
    }

    return drag;
}


void
RigidBody::_integrateLinear( float dt )
{
    const auto &conf = m_world.config;
    const auto  grav = m_gravscale * conf.gravity;

    glm::vec3 F = state.linear.F + computeAeroForcesLinear();

    state.linear.acc  = F * invMass + grav;
    state.linear.vel += dt*state.linear.acc;
    state.pos        += dt*state.linear.vel;
}


void
RigidBody::_integrateAngular( float dt )
{
    static constexpr float damping = 0.01f;

    glm::vec3 F = state.angular.F + computeAeroForcesAngular();

    state.angular.acc  = F;
    state.angular.vel += dt * state.angular.acc;
    state.angular.vel *= (1.0f - dt*damping);

    float magSq = glm::length2(state.angular.vel);
    if (magSq > 0.0f)
    {
        float mag = glm::sqrt(magSq);
        if (mag < 1e-6f)
        {
           return; 
        }

        auto  axis = state.angular.vel / mag;
        state.rot = glm::angleAxis(dt*mag, axis) * state.rot;
        state.rot = glm::normalize(state.rot);
    }
}



void
RigidBody::integrate( float dt )
{
    _onChange();
    _integrateLinear(dt);
    _integrateAngular(dt);
    updateTransforms();
}


void 
RigidBody::applyTranslation( const glm::vec3 &v )
{
    state.pos += v;;
}


void
RigidBody::applyImpulse( const glm::vec3 &point, const glm::vec3 &impulse )
{
    state.linear.vel += impulse;
    applyTorque(point, impulse);
}


void 
RigidBody::applyImpulseLinear( const glm::vec3 &J )
{
    state.linear.vel += J;
}


void 
RigidBody::applyImpulseAngular( const glm::vec3 &J )
{
    state.angular.vel += state.invT3 * J;
}


void
RigidBody::applyForceLinear( const glm::vec3 &F )
{
    state.linear.F += F;
}


void
RigidBody::applyForceAngular( const glm::vec3 &F )
{
    state.angular.F += state.invT3 * F;
}


void
RigidBody::applyTorque( const glm::vec3 &point, const glm::vec3 &F )
{
    // glm::vec3 P_local = glm::inverse(state.T) * glm::vec4(point, 1.0f);
    // glm::vec3 F_local = glm::inverse(state.T) * glm::vec4(F, 0.0f);

    // glm::vec3 r_local   = P_local - glm::vec3(0.0f);
    // glm::vec3 tau_local = glm::cross(r_local, F_local);
    // state.tau += tau_local;

    glm::vec3 r   = point - state.pos;
    glm::vec3 tau = glm::cross(r, F);
    state.angular.F += Inertia.inverseWorld * tau;
}


void
RigidBody::clearForces()
{
    state.linear.F  *= 0.0f;
    state.linear.J  *= 0.0f;
    state.angular.F *= 0.0f;
    state.angular.J *= 0.0f;
    state.tau       *= 0.0f;
}




void
RigidBody::setMass( float m )
{
    Body::setMass(m);

    // auto *P = dynamic_cast<RigidBody*>(m_parent);

    // if (P)
    // {
    //     P->_onChange();
    // }
}











void 
RigidBody::_defaultOnCollisionEnter( const CollisionInfo &info )
{
    if (dynamic_cast<StaticBody*>(info.B.body) == nullptr)
    {
        return;
    }

    auto &p = info.contactPoint;
    auto &N = info.A.normal;
    auto *A = dynamic_cast<RigidBody*>(info.A.body);
    auto *B = info.B.body;

    float mi1 = A->invMass;
    float mi2 = B->invMass;
    float e1 = A->getRestitution();
    float e2 = B->getRestitution();

    glm::vec3 Av  = A->state.linear.vel;
    glm::vec3 Bv  = B->state.linear.vel;
    glm::vec3 Vr  = Av - Bv;
    float     vj  = -(1.0f + e1) * glm::dot(Vr, N);
    glm::vec3 J   = vj / (mi1 + mi2) * N;
    

    A->applyImpulseLinear(mi1*J);
    // A->applyTranslation(-info.A.penetration);

    // B->applyImpulseLinear(mi2*J);

}
