// #include "verlet.hpp"
// #include <IDKPhysics/body/verletbody.hpp>
// #include <IDKPhysics/world.hpp>
// #include <libidk/idk_glm.hpp>
// #include <libidk/idk_assert.hpp>


// using namespace idk::phys;


// VerletConstraint::VerletConstraint( VerletBody *A, VerletBody *B, float maxdist )
// :   Constraint(A, B),
//     m_maxdist(maxdist)
// {
//     A->mNextBody = B;
//     B->mPrevBody = A;
// }


// void
// VerletConstraint::integrate( float dt )
// {
//     auto *A = dynamic_cast<VerletBody*>(m_A);
//     auto *B = dynamic_cast<VerletBody*>(m_B);

//     glm::vec3 disp = B->mCurrPos - A->mCurrPos;

//     if (glm::length2(disp) <= 1e-6f)
//     {
//         return;
//     }

//     float error = m_maxdist - glm::length(disp);
//     auto  dir   = error * glm::normalize(disp);

//     if (error > 0.0f)
//     {
//         return;
//     }

//     if (A->m_static)
//     {
//         B->mCurrPos += dir;
//     }

//     else
//     {
//         A->mCurrPos -= 0.5f * dir;
//         B->mCurrPos += 0.5f * dir;
//     }

// }