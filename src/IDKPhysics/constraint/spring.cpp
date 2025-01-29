#include "spring.hpp"
#include "../body/rigidbody.hpp"
#include <libidk/idk_glm.hpp>

using namespace idk::phys;


SpringConstraint::SpringConstraint( RigidBody *A, RigidBody *B,
                                    const glm::vec3 &offset, float Ck, float Cd )
:   Constraint(A, B),
    m_offset(offset),
    m_Ck(Ck),
    m_Cd(Cd)
{

}


void
SpringConstraint::integrate( float dt )
{
    glm::vec3 tgt  = m_A->state.T * glm::vec4(m_offset, 1.0f);
    glm::vec3 disp = m_B->state.pos - tgt;

    float M = 1.0f / (m_A->invMass + m_B->invMass);
    float Ck = m_Ck;
    float Cd = m_Cd;

    float dtSqInv = 1.0f / (dt*dt);
    float dtInv   = 1.0f / dt;

    auto  x = disp;
    auto  v = m_B->state.linear.vel - m_A->state.linear.vel;

    auto  F = -(dtSqInv*Ck*x + dtInv*Cd*v) * M;
    auto  J = -(dtInv*Ck*x + Cd*v) * M;


    m_A->applyImpulseLinear(-0.5f * J);
    m_B->applyImpulseLinear(+0.5f * J);

    // m_A->applyForceLinear(-0.5f * F);
    // m_B->applyForceLinear(+0.5f * F);
}


// void
// SpringConstraint::integrate( float dt )
// {
//     glm::vec3 tgt  = m_A->state.T * glm::vec4(m_offset, 1.0f);
//     glm::vec3 disp = m_B->state.pos - tgt;

//     float Am = m_A->getMass();
//     float Bm = m_B->getMass();

//     float Rm    = (Am*Bm) / (Am+Bm);
//     float k_max = Rm / (dt*dt);
//     float d_max = Rm / dt;

//     float Ck = m_Ck;
//     float Cd = m_Cd;

//     auto  x = disp;
//     auto  v = m_B->state.linear.vel - m_A->state.linear.vel;
//     auto  F = -k_max*Ck*x - d_max*Cd*v;

//     m_A->applyForceLinear(-0.5f * F);
//     m_B->applyForceLinear(+0.5f * F);
// }


AngularSpringConstraint::AngularSpringConstraint( RigidBody *A, RigidBody *B,
                                    const glm::vec3 &dir, float Ck, float Cd )
:   Constraint(A, B),
    m_dir(dir),
    m_Ck(Ck),
    m_Cd(Cd)
{

}


void
AngularSpringConstraint::integrate( float dt )
{
    // glm::vec3 tgt  = m_A->state.T * glm::vec4(m_offset, 1.0f);
    // glm::vec3 disp = m_B->state.pos - tgt;

    // float Am = m_A->getMass();
    // float Bm = m_B->getMass();
    // auto  Ai = m_A->Inertia;
    // auto  Bi = m_B->Inertia;

    // float m_red = (Am*Bm) / (Am+Bm);
    // auto  I_red = (Ai*Bi) / (Ai+Bi);
    // float Ck = m_Ck;
    // float Cd = m_Cd;
    // auto  x  = disp;
    // auto  v  = m_B->state.linear.vel - m_A->state.linear.vel;

    // {
    //     float k_max = m_red / (dt*dt);
    //     float d_max = m_red / dt;
    //     auto  F = -k_max*Ck*x - d_max*Cd*v;
    // }


    // auto rot =  = 1.0f;
    // // w (omega) -> angular velocity
    // auto w = m_B->state.angular.vel - m_A->state.angular.vel;

    // auto Jl = -(m_red/dt)*Ck*x - Cd*v;
    // auto Ja = -(I_red/dt)*Ck*theta - Cd*w;

    // m_A->applyForceLinear(-0.5f * F);
    // m_B->applyForceLinear(+0.5f * F);
}