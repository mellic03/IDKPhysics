#pragma once


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
    RigidBody *m_A;
    RigidBody *m_B;

public:
    Constraint( RigidBody *A, RigidBody *B )
    :   m_A(A), m_B(B)
    {
        
    }

    const int getID() const { return m_id; };
    const int getID()       { return m_id; };

    RigidBody *getBodyA() { return m_A; };
    RigidBody *getBodyB() { return m_B; };

    virtual void integrate( float dt ) = 0;
};
