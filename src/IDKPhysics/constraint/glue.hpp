#pragma once
#include "../common.hpp"
#include "constraint.hpp"
#include <glm/glm.hpp>

namespace idk::phys
{
    class GlueConstraint;
}


class idk::phys::GlueConstraint: public Constraint
{
private:
    glm::vec3 m_offset;

public:
    GlueConstraint( Body *A, Body *B, const glm::vec3 &offset );
    virtual void integrate( float dt ) final;
};
