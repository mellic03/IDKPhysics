#include "body.hpp"
#include "../shape/cuboid.hpp"

idk::phys::BodyState::BodyState( const glm::vec3 &p )
{
    pos = p;
    rot = glm::quat(glm::vec3(0.0f));
}


idk::phys::Body::Body( World &world, const glm::vec3 &pos, Shape *shape )
:   m_world(world),
    m_shape(shape),
    m_curr_state(pos),
    m_prev_state(pos),
    m_lerp_state(pos),
    m_pos(m_curr_state.pos),
    m_rot(m_curr_state.rot),
    m_extents(1.0f)
{
    if (m_shape == nullptr)
    {
        m_shape = new ShapeCuboid();
    }
}


glm::mat4
idk::phys::Body::getTransform()
{
    glm::mat4 T = glm::translate(glm::mat4(1), m_pos);
    glm::mat4 R = glm::mat4_cast(m_rot);
    glm::mat4 S = glm::scale(glm::mat4(1), m_shape->extents);
    return T*R*S;
}


