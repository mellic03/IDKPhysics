#include "glue.hpp"
#include "../body/rigidbody.hpp"
#include <libidk/idk_glm.hpp>

using namespace idk::phys;


GlueConstraint::GlueConstraint( Body *A, Body *B,
                                const glm::vec3 &offset )
:   Constraint(A, B),
    m_offset(offset)
{

}


void
GlueConstraint::integrate( float dt )
{
    // glm::mat4 T    = m_A->getTransform();
    // glm::vec4 xyzw = T * glm::vec4(m_offset, 1.0f);
    // m_B->setWorldPosition(glm::vec3(xyzw));
}