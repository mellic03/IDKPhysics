#pragma once
#include "constraint.hpp"
#include <glm/glm.hpp>

namespace idk::phys
{
    class SpringConstraint;
    class AngularSpringConstraint;
}


class idk::phys::SpringConstraint: public Constraint
{
public:
    glm::vec3 offset;
    float m_Ck, m_Cd;

    SpringConstraint( Body *A, Body *B, const glm::vec3 &offset,
                      float Ck=0.05f, float Cd=0.5f );
    virtual void integrate( float dt ) final;
};



class idk::phys::AngularSpringConstraint: public Constraint
{
public:
    glm::vec3 m_dir;
    float m_Ck, m_Cd;

    AngularSpringConstraint( Body *A, Body *B, const glm::vec3 &dir, float Ck=0.05f, float Cd=0.5f );
    virtual void integrate( float dt ) final;
};
