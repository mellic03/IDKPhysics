#include "world.hpp"
#include "query.hpp"
#include "body.hpp"
#include "collision/collision.hpp"


idk::phys::WorldConfig::WorldConfig()
{
    tickrate      = 60;
    substeps      = 1;
    fluid_density = 1.225f;
    gravity       = glm::vec3(0.0, 0.0, 0.0);
}


idk::phys::World::World( uint32_t maxBodies )
:   BodyManager (maxBodies),
    m_accum     (0.0f)
{
    phys::registerMethods();
}


idk::phys::World::~World()
{
    // for (auto *B: m_rigid_bodies)
    // {
    //     delete B;
    // }
}




void
idk::phys::World::update( float dt )
{
    const float timestep = 1.0f / float(config.tickrate);
    const int   substeps = config.substeps;
    // const float dt_substep = timestep / float(config.substeps);

    m_accum += dt;

    while (m_accum >= timestep)
    {
        for (int i=0; i<m_states.size(); i++)
        {
            // m_history[i].push_back(m_states[i]);

            // if (m_history[i].size() > 8)
            // {
            //     m_history[i].pop_front();
            // }
        }

        for (int i=0; i<substeps; i++)
        {
            _find_collisions(timestep);
            m_collisions.resolve();
            BodyManager::_integrate(timestep, substeps);
        }

        for (auto *B: m_rigid_bodies)
        {
            B->state.clearForces();
        }

        m_accum -= timestep;
    }


    const float alpha = glm::clamp(m_accum/timestep, 0.0f, 1.0f);

    for (auto *B: m_rigid_bodies)
    {
        // auto k = KinematicState::mix(history.back(), state, alpha);
        // m_lerp = KinematicState::mix(m_prev_state, state, alpha);

        B->update(alpha);
    }
}



bool
idk::phys::World::raycast( const glm::vec3 &ro, const glm::vec3 &rd,
                           glm::vec3 *hit, glm::vec3 *N, Body **body )
{
    glm::vec3 nearest_hit = glm::vec3(0.0f);
    glm::vec3 nearest_N   = glm::vec3(0.0f);
    float     nearest_dSq = INFINITY;

    // for (auto &wrapper: m_bodies)
    // {
    //     auto *B     = wrapper.body.get();
    //     auto &state = wrapper.state;

    //     if (B->getShape()->raycast(state, ro, rd, hit, N) == false)
    //     {
    //         continue;
    //     }

    //     float dSq = glm::distance2(state.pos, *hit);

    //     if (dSq < nearest_dSq)
    //     {
    //         nearest_hit = *hit;
    //         nearest_N   = *N;
    //         nearest_dSq = dSq;
    //         *body = B;
    //     }
    // }

    // if (nearest_dSq < INFINITY)
    // {
    //     *hit = nearest_hit;
    //     *N   = nearest_N;
    //     return true;
    // }

    return false;
}









void
idk::phys::World::_find_collisions( float dt )
{
    CollisionInfo info;

    for (RigidBody *A: BodyManager::rigidBodies())
    {
        for (StaticBody *B: BodyManager::staticBodies())
        {
            if (m_collisions.contains(A, B))
            {
                continue;
            }

            if (phys::detectCollisionShapeShape(&A->shape, &B->shape, info))
            {
                m_collisions.insert(info);
            }
        }
    }

    // for (int i=0; i<m_rigid_bodies.size(); i++)
    // {
    //     for (int j=i+1; j<m_rigid_bodies.size(); j++)
    //     {
    //         auto *A = m_rigid_bodies[i];
    //         auto *B = m_rigid_bodies[j];

    //         if (m_collisions.contains(A, B))
    //         {
    //             continue;
    //         }

    //         if (phys::detectCollisionShapeShape(&A->shape, &B->shape, info))
    //         {
    //             m_collisions.insert(info);
    //         }
    //     }
    // }

}