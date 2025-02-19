#include "body-manager.hpp"
#include "body.hpp"

using namespace idk::phys;


BodyManager::BodyManager( size_t maxBodies )
:   MAX_BODIES(maxBodies)
{
    m_bodies.reserve(maxBodies);
    // m_history.reserve(maxBodies);

    m_static_bodies.reserve(maxBodies);
    m_rigid_bodies.reserve(maxBodies);
    m_soft_bodies.reserve(maxBodies);

    m_states.reserve(maxBodies);
    m_static_states.reserve(maxBodies);
    m_rigid_states.reserve(maxBodies);
    // m_soft_states.reserve(maxBodies);
}


BodyManager::~BodyManager()
{
    for (auto *B: m_rigid_bodies)
    {
        delete B;
    }
}


void
BodyManager::updateBodies( float dt, int tickrate, int substeps )
{

}



void
BodyManager::_integrate( float dt, int substeps )
{
    dt /= float(substeps);
    // _find_collisions(dt);
    // m_collisions.resolve();

    // _applyGForces(dt, substeps);

    for (auto *B: m_rigid_bodies)
    {
        B->integrate(dt);
    }

    _integrateLinear(dt, substeps);
    _integrateAngular(dt, substeps);

    // for (auto *B: m_bodies)
    // {
    //     B->clearForces();
    // }

    for (auto &state: m_rigid_states)
    {
        state.updateTransforms();
    }

    for (auto &[id, C]: m_constraints)
    {
        if (C)
        {
            C->integrate(dt);
        }
    }

}



void
BodyManager::_applyGForces( float dt, int substeps )
{
    // // glm::vec3 G = (1.0f / config.substeps) * config.gravity;
    // glm::vec3 G = glm::vec3(0.0, -9.8, 0.0) / float(substeps);

    // for (auto &state: m_rigid_states)
    // {
    //     state.linear.dVel += G;
    //     // state.angular.dVel += G;
    // }
}


void
BodyManager::_integrateLinear( float dt, int substeps )
{
    // glm::vec3 G = (1.0f / substeps) * glm::vec3(0.0, -9.8, 0.0);
    glm::vec3 G = glm::vec3(0.0, -9.8, 0.0);

    for (auto &B: m_rigid_states)
    {
        float Mi = B.invMass;

        B.linear.acc  = Mi*B.linear.dAcc + G;
        B.linear.vel += dt*B.linear.acc + B.linear.dVel;
        B.pos        += dt*B.linear.vel + B.linear.dPos;

        // B.linear.dAcc   *= 0.0f;
        // B.linear.dVel *= 0.0f;
    }
}


void
BodyManager::_integrateAngular( float dt, int substeps )
{
    static constexpr float damping = 0.05f;
    float factor = (1.0f / substeps);
    float mag, magSq;

    for (auto &B: m_rigid_states)
    {
        B.angular.acc  = B.Iwi*B.angular.dAcc;
        B.angular.vel += dt*B.angular.acc + B.Iwi*B.angular.dVel;
        B.angular.vel *= (1.0f - dt*damping);

        // B.angular.dAcc   *= 0.0f;
        // B.angular.dVel *= 0.0f;

        magSq = glm::length2(B.angular.vel);
        if (magSq < 1e-12f)
        {
            B.angular.vel *= 0.0f;
            continue;
        }

        mag   = glm::sqrt(magSq);
        B.rot = glm::angleAxis(dt*mag, B.angular.vel/mag) * B.rot;
        B.rot = glm::normalize(B.rot);
    }
}
