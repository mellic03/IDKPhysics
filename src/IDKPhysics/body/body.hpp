#pragma once
#include "../common.hpp"
#include "../shape/shape.hpp"
#include "./state.hpp"
#include <deque>
#include <functional>


namespace idk::phys
{
    class CollisionInfo;
}


class idk::phys::Body
{
private:
    friend class World;
    int m_id;
    // std::deque<KinematicState> *history = nullptr;

protected:
    World    &m_world;
    float     m_staticCoefficient = 1.0f;

public:
    KinematicState &state;
    Shape           shape;

    std::function<void(const CollisionInfo&)> onCollisionEnter = [](const CollisionInfo&){};
    std::function<void(const CollisionInfo&)> onCollisionExit  = [](const CollisionInfo&){};

    Body( World&, KinematicState&, phys::shape_type stype = SHAPE_SPHERE );
    Body( World&, KinematicState&, const Shape& );
    virtual ~Body() = default;

    const int getID() const { return m_id; };
    const int getID()       { return m_id; };


    virtual void update( float alpha );
    virtual void integrate( float dt );

    World &getWorld() { return m_world; };

    template <typename T=idk::phys::Shape>
    T *getShape() { return dynamic_cast<T*>(shape); };

    glm::mat4 getRenderMatrix( bool scale=false );


    // KinematicState &operator->() { return state; }
    // const KinematicState &operator->() const { return state; }

    // auto applyTranslation = state::applyTranslation;

    float getMass();
    void  setMass( float m );

    void  applyTranslation( const glm::vec3& );

    void  applyImpulse( const glm::vec3 &worldVel, const glm::vec3 &worldPoint );
    void  applyImpulseLinear( const glm::vec3 &worldVel );
    void  applyImpulseAngular( const glm::vec3 &worldAxis );
    void  applyImpulseAngular( const glm::vec3 &worldVel, const glm::vec3 &worldPoint );

    void  applyForce( const glm::vec3 &worldAcc, const glm::vec3 &worldPoint );
    void  applyForceLinear( const glm::vec3 &worldAcc );
    void  applyForceAngular( const glm::vec3 &worldAxis );
    void  applyForceAngular( const glm::vec3 &worldAcc, const glm::vec3 &worldPoint );

    glm::vec3 computeTorque( const glm::vec3 &worldAcc, const glm::vec3 &worldPoint );

    void  updateTransforms();
    void  clearForces();

};




// template <typename body_type>
// struct idk::phys::Body
// {
// private:
//     friend class phys::World;
//     inline static KinematicState dummy;

// public:
//     Body       *impl;
//     Shape          *shape;
//     KinematicState &state;

//     Body( body_type *B, KinematicState &st = Body::dummy )
//     :   impl  (dynamic_cast<Body*>(B)),
//         state (st),
//         shape (impl->shape)
//     {

//     }

//     const int getID() const { return impl->m_id; };
//     const int getID()       { return impl->m_id; };

//     glm::mat4 getRenderMatrix( bool scale=false )
//     {
//         return state.T;
//         // state.
//     }


//     template <typename T>
//     Body<T> cast()
//     {
//         return Body<T>(dynamic_cast<T*>(impl), state);
//     }

//     template <typename T>
//     const Body<T> cast() const
//     {
//         return Body<T>(dynamic_cast<T*>(impl), state);
//     }
// };
