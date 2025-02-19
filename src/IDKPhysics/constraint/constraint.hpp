#pragma once
#include "../body/body.hpp"

namespace idk::phys
{
    class RigidBody;
    class Constraint;
}


class idk::phys::Constraint
{
private:
    friend class World;
    int m_id;

protected:
    Body *m_A;
    Body *m_B;

public:

    Constraint( Body *A, Body *B )
    :   m_A(A), m_B(A)
    {
        
    }

    const int getID() const { return m_id; };
    const int getID()       { return m_id; };

    // RigidBody *getBodyA() { return m_A; };
    // RigidBody *getBodyB() { return m_B; };

    virtual void integrate( float dt ) = 0;
};
