// #include "distance.hpp"
// #include "../body/rigidbody.hpp"
// #include <libidk/idk_glm.hpp>

// using namespace idk::phys;


// DistanceConstraint::DistanceConstraint( RigidBody *A, RigidBody *B, float dist, float damp )
// :   Constraint(A, B),
//     target_dist(dist),
//     damping(damp)
// {

// }


// void
// DistanceConstraint::integrate( float dt )
// {
//     // auto &Apos = m_A->getState().pos;
//     // auto &Bpos = m_B->getState().pos;

//     // glm::vec3 disp = Bpos - Apos;
//     // float curr_dist = glm::length(disp);

//     // if (curr_dist == 0.0f) return; // Prevent divide-by-zero.

//     // glm::vec3 dir = glm::normalize(disp);
//     // float delta_dist = curr_dist - this->target_dist;

//     // float cutoff = 0.25f;
//     // float scale  = glm::clamp(glm::abs(delta_dist) / cutoff, 0.0f, 1.0f);


//     // // Correct velocities via impulses
//     // glm::vec3 rel_vel = m_B->getVelocity() - m_A->getVelocity();
//     // float rel_speed = glm::dot(rel_vel, dir);

//     // // Damping correction
//     // float impulse_mag = -delta_dist / dt - this->damping * rel_speed;
//     // glm::vec3 impulse = impulse_mag * dir;

//     // if (!m_A->m_static && !m_B->m_static)
//     // {
//     //     m_A->getVelocity() -= scale * impulse / m_A->getMass();
//     //     m_B->getVelocity() += scale * impulse / m_B->getMass();
//     // }

//     // else if (m_A->m_static)
//     // {
//     //     m_B->getVelocity() += scale * (impulse / m_B->getMass());
//     // }

//     // else if (m_B->m_static)
//     // {
//     //     m_A->getVelocity() -= scale * (impulse / m_A->getMass());
//     // }

// }