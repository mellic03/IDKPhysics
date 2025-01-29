#pragma once
#include "../common.hpp"
#include "../shape/shape.hpp"
#include "./state.hpp"
#include <functional>


class idk::phys::BodyState
{
public:
    BodyState( const glm::vec3 &p );

    glm::vec3 pos;
    glm::quat rot;
};



class idk::phys::Body
{
private:
    friend class World;
    int m_id;
    glm::vec3 m_up, m_right, m_front;

protected:
    // enum prop_idx {
    //     IDX_MASS       = 0,
    //     IDX_DRAG       = 1,
    //     IDX_RESTITUION = 2,
    //     IDX_GRAVSCALE  = 3
    // };

    // enum prop_bit {
    //     BIT_MASS       = 1<<IDX_MASS,
    //     BIT_DRAG       = 1<<IDX_DRAG,
    //     BIT_RESTITUION = 1<<IDX_RESTITUION,
    //     BIT_GRAVSCALE  = 1<<IDX_GRAVSCALE
    // };

    // uint32_t m_bitflags = 0; 
    float m_mass, m_drag, m_restitution, m_gravscale;

    World    &m_world;
    Shape    *m_shape;

    KinematicState m_prev_state;
    KinematicState m_lerp, m_prevlerp;


public:
    // KinematicState lspace, state;
    KinematicState state;

    // Body *m_parent = nullptr;
    float invMass;

    std::function<void(const CollisionInfo&)> onCollisionEnter = [](const CollisionInfo&){};
    std::function<void(const CollisionInfo&)> onCollisionExit  = [](const CollisionInfo&){};

    Body( World&, const glm::vec3 &p=glm::vec3(0), Shape *shape=nullptr );
    virtual ~Body() = default;

    virtual void update( float alpha );
    virtual void swapStates();

    void  updateTransforms();
    bool  collides( Body*, CollisionInfo *info = nullptr );
    bool  raycast( const glm::vec3 &ro, const glm::vec3 &rd, glm::vec3 *hit, glm::vec3 *N );

    const int getID() const { return m_id; };
    const int getID()       { return m_id; };

    World &getWorld() { return m_world; };

    template <typename T=idk::phys::Shape>
    T *getShape() { return dynamic_cast<T*>(m_shape); };

    glm::mat4 getRenderMatrix( bool scale=false );
    glm::mat4 getRenderMatrixPrev( bool scale=false );

    const glm::vec3 &getUp()    { return m_up; }
    const glm::vec3 &getRight() { return m_right; }
    const glm::vec3 &getFront() { return m_front; }


    virtual void  setMass( float ); 
    virtual void  setDrag( float ); 
    virtual void  setRestitution( float ); 
    virtual void  setGravScale( float );

    virtual float getMass();
    virtual float getDrag();
    virtual float getRestitution();
    virtual float getGravScale();
};
