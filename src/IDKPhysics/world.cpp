#include "world.hpp"
#include "query.hpp"
#include "body.hpp"


idk::phys::WorldConfig::WorldConfig()
{
    tickrate      = 60;
    substeps      = 4;
    fluid_density = 1.225f;
    gravity       = glm::vec3(0.0, 0.0, 0.0);
}


idk::phys::World::World()
:   m_accum(0.0f)
{
    // T = -131.21 + 0.00299*h
    // density ~= p / (0.2869 * (T + 273.1))

}


idk::phys::World::~World()
{
    for (auto *B: m_rigid_bodies)
    {
        delete B;
    }
}


void
idk::phys::World::_find_collisions( float dt )
{
    m_collision_list.clear();
    // _find_collisions(m_rigid_bodies, m_rigid_bodies);
    _find_collisions(m_rigid_bodies, m_static_bodies);
}


void
idk::phys::World::_resolve_collisions( float dt )
{
    for (auto &info: m_collision_list)
    {
        info.A.body->onCollisionEnter(info);
        std::swap(info.A, info.B);
        info.A.body->onCollisionEnter(info);
    }
}


void
idk::phys::World::_integrate( float dt )
{
    _find_collisions(dt);
    _resolve_collisions(dt);

    for (auto *B: m_rigid_bodies)
    {
        B->integrate(dt);
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
idk::phys::World::update( float dt )
{
    const float timestep   = 1.0f / float(config.tickrate);
    const float dt_substep = timestep / float(config.substeps);

    m_accum += dt;
    // m_accum = glm::min(m_accum+dt, 4.0f*timestep);

    while (m_accum >= timestep)
    {
        for (auto *B: m_rigid_bodies)
        {
            B->swapStates();
        }

        for (int i=0; i<config.substeps; i++)
        {
            _integrate(dt_substep);
        }

        for (auto *B: m_rigid_bodies)
        {
            B->clearForces();
        }

        m_accum -= timestep;
    }


    const float alpha = glm::clamp(m_accum/timestep, 0.0f, 1.0f);

    for (auto *B: m_rigid_bodies)
    {
        B->update(alpha);
    }
}


void
idk::phys::World::deleteConstraint( int id )
{
    m_constraints.destroy(id);
}



bool
idk::phys::World::raycast( const glm::vec3 &ro, const glm::vec3 &rd,
                           glm::vec3 *hit, glm::vec3 *N, Body **body )
{
    glm::vec3 nearest_hit = glm::vec3(0.0f);
    glm::vec3 nearest_N   = glm::vec3(0.0f);
    float     nearest_dSq = INFINITY;

    for (auto *B: m_bodies)
    {
        if (B->raycast(ro, rd, hit, N) == false)
        {
            continue;
        }

        float dSq = glm::distance2(B->state.pos, *hit);

        if (dSq < nearest_dSq)
        {
            nearest_hit = *hit;
            nearest_N   = *N;
            nearest_dSq = dSq;
            *body = B;
        }
    }

    if (nearest_dSq < INFINITY)
    {
        *hit = nearest_hit;
        *N   = nearest_N;
        return true;
    }

    return false;
}
