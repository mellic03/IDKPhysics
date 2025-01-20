#pragma once
#include "../common.hpp"
#include "body.hpp"


class idk::phys::RigidBody: public idk::phys::Body
{
private:
    friend class World;

    glm::vec3 m_vel;
    glm::vec3 m_acc;
    glm::vec4 m_ang_vel;
    glm::vec4 m_ang_acc;

    glm::vec3 m_forces;
    glm::vec4 m_ang_forces;

public:
          RigidBody( idk::phys::World&, const glm::vec3&, Shape *shape );
         ~RigidBody();
    void  update( float alpha );

    /** @param dt delta time in milliseconds. */
    void  integrate( float dt );

    void  addForce( const glm::vec3 &F ) { m_forces += F; };
    void  mulForce( const glm::vec3 &F ) { m_forces *= F; };

    void  addForceAngular( const glm::vec3 &F, float mag );
    void  mulForceAngular( const glm::vec3 &F );

    const glm::vec3 &getPosition() const { return m_lerp_state.pos; };
};
