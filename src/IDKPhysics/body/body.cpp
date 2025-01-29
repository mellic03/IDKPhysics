#include "body.hpp"
#include "../shape/sphere.hpp"
#include "../shape/OBB.hpp"
#include <libidk/idk_glm.hpp>

using namespace idk::phys;



BodyState::BodyState( const glm::vec3 &p )
{
    pos = p;
    rot = glm::quat(glm::vec3(0.0f));
}


Body::Body( World &world, const glm::vec3 &pos, Shape *shape )
:   m_world       (world),
    m_shape       (shape),
    state         (pos, glm::quat(glm::vec3(0.0f))),
    m_lerp        (pos, glm::quat(glm::vec3(0.0f))),
    m_prev_state  (pos, glm::quat(glm::vec3(0.0f))),
    m_prevlerp    (pos, glm::quat(glm::vec3(0.0f))),
    m_mass        (1.0f),
    m_drag        (0.05f),
    m_restitution (0.66f),
    m_gravscale   (1.0f)
{
    invMass = 1.0f / m_mass;

    m_up    = glm::vec3(0, 1, 0);
    m_right = glm::vec3(1, 0, 0);
    m_front = glm::vec3(0, 0, -1);

    if (m_shape == nullptr)
    {
        m_shape = new ShapeSphere();
    }

    m_shape->body = this;

    updateTransforms();
}


void
Body::update( float alpha )
{
    m_prevlerp = m_lerp;

    // KinematicState temp = lspace;
    // temp.pos = Tworld.T * glm::vec4(lspace.pos, 1.0f);
    // temp.rot = glm::quat_cast(Tworld.T * glm::mat4_cast(lspace.rot));
    // temp.rot = glm::normalize(temp.rot);

    m_lerp = KinematicState::mix(m_prev_state, state, alpha);
}


bool
idk::phys::Body::collides( Body *B, CollisionInfo *info )
{
    return m_shape->collides(B->getShape(), info);
}




void
idk::phys::Body::swapStates()
{
    m_prev_state = state;
}


void
idk::phys::Body::updateTransforms()
{
    auto Tlocal = glm::translate(glm::mat4(1.0f), state.pos);
    auto Rlocal = glm::mat4_cast(state.rot);

    state.T     = Tlocal * Rlocal;
    state.T3    = glm::mat3(state.T);
    state.invT  = glm::inverse(state.T);
    state.invT3 = glm::inverse(state.T3);

    m_up    = glm::normalize(state.T3 * glm::vec3( 0, +1,  0));
    m_right = glm::normalize(state.T3 * glm::vec3(+1,  0,  0));
    m_front = glm::normalize(state.T3 * glm::vec3( 0,  0, +1));


    // local.T       = wspace.T;
    // local.pos     = wspace.pos;
    // local.linear  = wspace.linear;
    // local.angular = wspace.angular;

    // if (m_parent)
    // {
    //     const auto &plocal = m_parent->local;

    //     local.linear.acc += plocal.linear.acc;
    //     local.linear.vel += plocal.linear.vel;
    // }
}



glm::mat4
idk::phys::Body::getRenderMatrix( bool scale )
{
    auto S = (scale) ? glm::mat4(1) : glm::scale(glm::mat4(1), m_shape->extents);
    return m_lerp.T * S;
}

glm::mat4
idk::phys::Body::getRenderMatrixPrev( bool scale )
{
    auto S = (scale) ? glm::mat4(1) : glm::scale(glm::mat4(1), m_shape->extents);
    return m_prevlerp.T * S;
}


bool
idk::phys::Body::raycast( const glm::vec3 &ro, const glm::vec3 &rd,
                          glm::vec3 *hit, glm::vec3 *N )
{
    return m_shape->raycast(ro, rd, hit, N);
}





// void
// Body::setProperty( uint32_t idx, float value )
// {
//     m_properties[idx] = value;
//     m_bitflags |= (1<<idx);
// }


// float&
// Body::getProperty( uint32_t idx )
// {
//     return m_properties[idx];
// }



void Body::setMass( float m )
{
    m_mass  = m;
    invMass = 1.0f / m;
}

void Body::setDrag( float f )        { m_drag = f; }
void Body::setRestitution( float f ) { m_restitution = f; }
void Body::setGravScale( float f )   { m_gravscale = f; }


float Body::getMass()        { return m_mass; }
float Body::getDrag()        { return m_drag; }
float Body::getRestitution() { return m_restitution; }
float Body::getGravScale()   { return m_gravscale; }

