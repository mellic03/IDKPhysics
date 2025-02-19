#include "body.hpp"
#include "../world.hpp"
#include "../shape/sphere.hpp"
#include "../shape/OBB.hpp"
#include <libidk/idk_glm.hpp>

using namespace idk::phys;



Body::Body( World &world, KinematicState &st, shape_type stype )
:   m_world(world),
    state(st),
    shape(stype)
{
    shape.body = this;

    // updateTransforms();
}


Body::Body( World &world, KinematicState &st, const Shape &S )
:   m_world(world),
    state(st),
    shape(S)
{
    shape.body = this;
}


void
Body::update( float alpha )
{

}


void
Body::integrate( float dt )
{

}


// bool
// Body::collides( Body *B, CollisionInfo *info )
// {
//     return shape->collides(B->getShape(), info);
// }


// void
// Body::updateTransforms()
// {
//     state.T     = glm::translate(glm::mat4(1), state.pos) * glm::mat4_cast(state.rot);
//     state.Ti    = glm::inverse(state.T);
//     state.up    = state.T * glm::vec4(0, 1, 0, 0);
//     state.right = state.T * glm::vec4(1, 0, 0, 0);
//     state.front = state.T * glm::vec4(0, 0, 1, 0);
// }



glm::mat4
Body::getRenderMatrix( bool scale )
{
    auto S = (scale) ? glm::scale(glm::mat4(1), shape.extents) : glm::mat4(1);
    return state.T * S;
}


float Body::getMass() { return state.getMass(); }
void  Body::setMass( float m ) { state.setMass(m); }

void
Body::applyTranslation( const glm::vec3 &v )
{
    state.applyTranslation(v);
}

void
Body::applyImpulse( const glm::vec3 &Jworld, const glm::vec3 &Pworld )
{
    state.applyImpulse(Jworld, Pworld);
}

void
Body::applyImpulseLinear( const glm::vec3 &Jworld )
{
    state.applyImpulseLinear(Jworld);
}

void
Body::applyImpulseAngular( const glm::vec3 &worldAxis )
{
    state.applyImpulseAngular(worldAxis);
}

void
Body::applyImpulseAngular( const glm::vec3 &Jworld, const glm::vec3 &Pworld )
{
    state.applyImpulseAngular(Jworld, Pworld);
}

void
Body::applyForce( const glm::vec3 &Fworld, const glm::vec3 &Pworld )
{
    state.applyForce(Fworld, Pworld);
}

void
Body::applyForceLinear( const glm::vec3 &Fworld )
{
    state.applyForceLinear(Fworld);
}

void
Body::applyForceAngular( const glm::vec3 &worldAxis )
{
    state.applyForceAngular(worldAxis);
}

void
Body::applyForceAngular( const glm::vec3 &Fworld, const glm::vec3 &Pworld )
{
    state.applyForceAngular(Fworld, Pworld);
}

glm::vec3
Body::computeTorque( const glm::vec3 &Fworld, const glm::vec3 &Pworld )
{
    return state.computeTorque(Fworld, Pworld);
}

void
Body::updateTransforms()
{
    state.updateTransforms();
}

void
Body::clearForces()
{
    state.clearForces();
}

