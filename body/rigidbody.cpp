#include "rigidbody.hpp"
#include "../world.hpp"
#include <libidk/idk_math.hpp>


idk::phys::RigidBody::RigidBody( World &world, const glm::vec3 &pos, Shape *shape )
:   Body(world, pos, shape)
{
    m_vel     = glm::vec3(0.0f);
    m_acc     = glm::vec3(0.0f);
    m_ang_vel = glm::vec4(0.0f);
    m_ang_acc = glm::vec4(0.0f);
}


idk::phys::RigidBody::~RigidBody()
{

}


void
idk::phys::RigidBody::update( float alpha )
{
    const auto &A = m_prev_state;
    const auto &B = m_curr_state;
    // m_lerp_state.pos = glm::mix(A.pos, B.pos, alpha);
    // m_lerp_state.rot = glm::mix(A.rot, B.rot, alpha);
    m_lerp_state.pos = B.pos;
    m_lerp_state.rot = B.rot;
}


void
idk::phys::RigidBody::integrate( float dt )
{
    const auto &conf = m_world.config;

    float area = 1.0f;
    auto  shape = m_shape->getType();

    if (shape == Shape::Type::SPHERE)
    {
        float r = m_shape->extents.x;
        area = idk::PI * r*r;
    }

    else if (shape == Shape::Type::CUBOID)
    {
        float r = m_shape->extents.x;
        area = idk::PI * r*r;
    }

    glm::vec3 drag = conf.drag_coef * conf.fluid_density * area * 0.5f * m_vel*m_vel;

    m_prev_state = m_curr_state;

    m_acc = m_forces + m_world.config.gravity - drag;

    m_vel += dt * m_acc;
    m_pos += dt * m_vel;

    // m_vel = idk::flerp(m_vel, glm::vec3(0), dt, 0.15f);
}






void
idk::phys::RigidBody::addForceAngular( const glm::vec3 &F, float mag )
{
    m_ang_forces.x += mag * F.x;
    m_ang_forces.y += mag * F.y;
    m_ang_forces.z += mag * F.z;
    m_ang_forces.w += mag;
}


void
idk::phys::RigidBody::mulForceAngular( const glm::vec3 &F )
{
    // m_ang_forces *= F;
}



