#include "world.hpp"


idk::phys::World::World( float timestep )
:   m_accum(0.0f),
    m_timestep(timestep)
{

}


idk::phys::World::~World()
{
    for (auto *B: m_rigid_bodies)
    {
        delete B;
    }
}


void
idk::phys::World::update( float dt )
{
    m_accum += dt;

    while (m_accum >= m_timestep)
    {
        for (auto *B: m_rigid_bodies)
        {
            B->integrate(m_timestep);
        }

        m_accum -= m_timestep;
    }


    const float alpha = glm::min(m_accum/m_timestep, 1.0f);

    for (auto *B: m_rigid_bodies)
    {
        B->update(alpha);
    }
}

