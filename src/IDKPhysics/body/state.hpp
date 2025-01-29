#pragma once
#include "../common.hpp"


namespace idk::phys
{
    struct KinematicState;
    struct TransformState;
    struct KinematicForce;
    struct CollisionInfo;
}



struct idk::phys::KinematicState
{
private:
    struct private_type
    {
        glm::vec3 F   = glm::vec3(0.0f);
        glm::vec3 J   = glm::vec3(0.0f);
        glm::vec3 acc = glm::vec3(0.0f);
        glm::vec3 vel = glm::vec3(0.0f);
    };

public:
    private_type linear;
    private_type angular;
    glm::vec3    tau = glm::vec3(0.0f);
    glm::vec3    pos;
    glm::quat    rot;
    glm::mat4    T     = glm::mat4(1.0f);
    glm::mat4    invT  = glm::mat4(1.0f);
    glm::mat3    T3    = glm::mat3(1.0f);
    glm::mat3    invT3 = glm::mat3(1.0f);

    KinematicState( const glm::vec3 &p, const glm::quat &r=glm::quat(glm::vec3(0.0f)) );

    static KinematicState mix( const KinematicState&, const KinematicState&, float );
};


struct idk::phys::TransformState
{
private:
    void _update();

public:
    glm::mat4 T     = glm::mat4(1.0f);
    glm::mat4 invT  = glm::mat4(1.0f);
    glm::mat3 T3    = glm::mat3(1.0f);
    glm::mat3 invT3 = glm::mat3(1.0f);

    void update( const glm::vec3 &pos, const glm::quat &rot );
    void update( const TransformState &parent, const glm::vec3 &pos, const glm::quat &rot );
    void update( const TransformState &parent, const TransformState &other );
};


struct idk::phys::KinematicForce
{
private:
    struct private_type
    {
        // Force in world space
        glm::vec3 F = glm::vec3(0.0f);
        // Impulse in world space
        glm::vec3 J = glm::vec3(0.0f);
    };

public:
    // World space linear
    private_type linear;
    // World space angular
    private_type angular;
    glm::vec3 torque = glm::vec3(0.0f);
};



struct idk::phys::CollisionInfo
{
    // struct response_type
    // {
    //     glm::vec3 force   = glm::vec3(0.0f);
    //     glm::vec3 impulse = glm::vec3(0.0f);
    //     glm::vec3 delta   = glm::vec3(0.0f);
    // };

    struct private_type
    {
        Body *body;
        /** Collision normal. */
        glm::vec3 normal;
        glm::vec3 penetration;
        // /** Response force/impulse. */
        // response_type response;
    };

public:
    /** Contact point. */
    glm::vec3 contactPoint;
    /** Info from A's perspective */
    private_type A;
    /** Info from B's perspective */
    private_type B;
};