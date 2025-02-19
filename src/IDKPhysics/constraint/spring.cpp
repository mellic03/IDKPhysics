#include "spring.hpp"
#include "../body/rigidbody.hpp"
#include <libidk/idk_glm.hpp>

using namespace idk::phys;


SpringConstraint::SpringConstraint( Body *A, Body *B, const glm::vec3 &offset, float Ck, float Cd )
:   Constraint(A, B),
    offset(offset),
    m_Ck(Ck),
    m_Cd(Cd)
{
    B->state.pos = A->state.T * glm::vec4(offset, 1.0f);
}


void
SpringConstraint::integrate( float dt )
{
    glm::vec3 tgt  = m_A->state.T * glm::vec4(offset, 1.0f);
    glm::vec3 disp = m_B->state.pos - tgt;

    float mi1 = m_A->state.invMass;
    float mi2 = m_B->state.invMass;
    float M   = 1.0f / (mi1 + mi2);

    float Ck = m_Ck;
    float Cd = m_Cd;

    float dtSqInv = 1.0f / (dt*dt);
    float dtInv   = 1.0f / dt;

    auto  x = disp;
    auto  v = m_B->state.linear.vel - m_A->state.linear.vel;
    auto  F = -M * ( dtSqInv*Ck*x  +  dtInv*Cd*v );
    auto  J = -M * ( dtInv  *Ck*x  +        Cd*v );

    m_A->state.applyImpulse(-0.5f*J, m_B->state.COM_world);
    m_B->state.applyImpulse(+0.5f*J, m_A->state.COM_world);

    m_A->state.applyForceLinear(-0.5f*F);
    m_B->state.applyForceLinear(+0.5f*F);

    // m_A->applyImpulseLinear(-0.5f*J);
    // m_B->applyImpulseLinear(+0.5f*J);
    // m_A->applyImpulseAngular(-0.5f*J, m_B->state.COM);
    // m_B->applyImpulseAngular(+0.5f*J, m_A->state.COM);

    // glm::vec3 rA = tgt - m_A->state.COM; // Offset from A's COM to target
    // glm::vec3 rB = m_B->state.pos - m_B->state.COM; // Offset from B's COM

    // glm::vec3 torqueA = glm::cross(rA, -0.5f * J);
    // glm::vec3 torqueB = glm::cross(rB, +0.5f * J);

    // m_A->applyImpulseAngular(torqueA);
    // m_B->applyImpulseAngular(torqueB);

}




// AngularSpringConstraint::AngularSpringConstraint( RigidBody *A, RigidBody *B,
//                                     const glm::vec3 &dir, float Ck, float Cd )
// :   Constraint(A, B),
//     m_dir(dir),
//     m_Ck(Ck),
//     m_Cd(Cd)
// {

// }


// void
// AngularSpringConstraint::integrate( float dt )
// {
//     // glm::vec3 tgt  = m_A->state.T * glm::vec4(m_offset, 1.0f);
//     // glm::vec3 disp = m_B->state.pos - tgt;

//     // float Am = m_A->getMass();
//     // float Bm = m_B->getMass();
//     // auto  Ai = m_A->Inertia;
//     // auto  Bi = m_B->Inertia;

//     // float m_red = (Am*Bm) / (Am+Bm);
//     // auto  I_red = (Ai*Bi) / (Ai+Bi);
//     // float Ck = m_Ck;
//     // float Cd = m_Cd;
//     // auto  x  = disp;
//     // auto  v  = m_B->state.linear.vel - m_A->state.linear.vel;

//     // {
//     //     float k_max = m_red / (dt*dt);
//     //     float d_max = m_red / dt;
//     //     auto  F = -k_max*Ck*x - d_max*Cd*v;
//     // }


//     // auto rot =  = 1.0f;
//     // // w (omega) -> angular velocity
//     // auto w = m_B->state.angular.vel - m_A->state.angular.vel;

//     // auto Jl = -(m_red/dt)*Ck*x - Cd*v;
//     // auto Ja = -(I_red/dt)*Ck*theta - Cd*w;

//     // m_A->applyForceLinear(-0.5f * F);
//     // m_B->applyForceLinear(+0.5f * F);
// }