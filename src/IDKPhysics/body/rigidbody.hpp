#pragma once
#include "../common.hpp"
#include "body.hpp"


class idk::phys::RigidBody: public idk::phys::Body
{
private:
    glm::vec3  m_COM;
    float      m_totalMass;

    float     _computeTotalMass();
    glm::vec3 _computeCOM( float totalMass );
    glm::mat3 _computeIntertiaTensor( const glm::vec3& );
    void      _onChange();

    struct private_type
    {
        glm::mat3 local, inverse, world, inverseWorld;
    };


protected:
    friend class World;

    virtual void _integrateLinear( float dt );
    virtual void _integrateAngular( float dt );

    void _defaultOnCollisionEnter( const CollisionInfo& );


public:
    bool m_static = false;
    private_type Inertia;
    std::vector<RigidBody*> m_children;

          RigidBody( idk::phys::World&, const glm::vec3 &p=glm::vec3(0), Shape *shape = nullptr );

    virtual void update( float alpha ) override;
    virtual void integrate( float dt );

    virtual glm::vec3 computeAeroForcesLinear();
    virtual glm::vec3 computeAeroForcesAngular();


    // void  addChild( RigidBody* );
    void  applyTranslation( const glm::vec3& );

    void  applyImpulse( const glm::vec3 &origin, const glm::vec3 &I );
    void  applyImpulseLinear( const glm::vec3 &I );
    void  applyImpulseAngular( const glm::vec3 &I );

    void  applyForceLinear( const glm::vec3 &F );
    void  applyForceAngular( const glm::vec3 &axis );
    void  applyTorque( const glm::vec3 &origin, const glm::vec3 &F );
    
    void  clearForces();

    virtual void setMass( float ) override;
};
