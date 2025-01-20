#pragma once
#include "../common.hpp"
#include "../shape/shape.hpp"

class idk::phys::BodyState
{
public:
    BodyState( const glm::vec3 &p );

    glm::vec3 pos;
    glm::quat rot;
};


class idk::phys::Body
{
protected:
    World     &m_world;
    Shape    *m_shape;

    BodyState m_curr_state;
    BodyState m_prev_state;
    BodyState m_lerp_state;

    glm::vec3 &m_pos;
    glm::quat &m_rot;
    glm::vec3 m_extents;

public:
    Body( World&, const glm::vec3&, Shape *shape = nullptr );
    glm::mat4 getTransform();
};
