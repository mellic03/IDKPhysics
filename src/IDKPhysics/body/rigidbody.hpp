#pragma once
#include "../common.hpp"
#include "body.hpp"


class idk::phys::RigidBody: public idk::phys::Body
{
private:
    // float     _computeTotalMass();
    // glm::vec3 _computeCOM( float totalMass );
    // glm::mat3 _computeIntertiaTensor( const glm::vec3& );
    // void      _onChange();


protected:
    friend class World;

    // virtual void _integrateLinear( float dt, float substep_factor=1.0f );
    // virtual void _integrateAngular( float dt, float substep_factor=1.0f );
    // void _defaultOnCollisionEnter( const CollisionInfo& );


public:
    bool m_static = false;
    RigidBody* m_parent;
    std::vector<RigidBody*> m_children;

    RigidBody( idk::phys::World&, KinematicState&, const phys::Shape& );
    RigidBody( idk::phys::World&, KinematicState&, phys::shape_type stype = SHAPE_SPHERE );

    virtual void update( float alpha ) override;
    virtual void integrate( float dt ) override;

    virtual std::pair<glm::vec3, glm::vec3> computeGravitationalAcceleration();
    virtual glm::vec3 computeAeroForcesLinear();
    virtual glm::vec3 computeAeroForcesAngular();


    // void  addChild( RigidBody* );

    // virtual void setMass( float ) override;
};
