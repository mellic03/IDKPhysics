#include "./common.hpp"
#include "constraint/constraint.hpp"
#include "body/body.hpp"

#include <vector>
#include <iostream>

#include <libidk/idk_wallocator2.hpp>


class idk::phys::WorldConfig
{
public:
    WorldConfig();
    int   tickrate;
    int   substeps;
    float fluid_density;
    glm::vec3 gravity;
};



class idk::phys::World
{
private:
    float m_accum;

    std::vector<Body*>         m_bodies;
    std::vector<StaticBody*>   m_static_bodies;
    std::vector<RigidBody*>    m_rigid_bodies;
    std::vector<SoftBody*>     m_soft_bodies;

    idk::WAllocator2<Constraint>  m_constraints;

    std::vector<CollisionInfo> m_collision_list;

    template <typename T, typename U>
    void _find_collisions( std::vector<T>&, std::vector<U>& );

    void _find_collisions( float dt );
    void _resolve_collisions( float dt );
    void _integrate( float dt );

    template <typename T>
    void _pushBody( Body *B, std::vector<T*> &dst )
    {
        m_bodies.push_back(dynamic_cast<Body*>(B));
        B->m_id = m_bodies.size() - 1;
        dst.push_back(dynamic_cast<T*>(B));
    }


public:
    WorldConfig config;

          World();
         ~World();
    void  update( float dt );

    bool raycast( const glm::vec3 &ro, const glm::vec3 &rd, glm::vec3 *hit, glm::vec3 *N, Body **B );

    const auto &rigidBodies()  const { return m_rigid_bodies;  };
    const auto &staticBodies() const { return m_static_bodies; };
    const auto &constraints()  const { return m_constraints;   };

    // int addConstraint( Constraint* );
    void deleteConstraint( int id );

    template <typename body_type, typename... Args>
    body_type *createBody( Args&&... );

    template <typename constraint_type, typename... Args>
    constraint_type *createConstraint( Args&&... );

};


template <typename T, typename... Args>
T*
idk::phys::World::createBody( Args&&... args )
{
    auto *B = new T(*this, std::forward<Args>(args)...);

    if      constexpr (std::is_base_of_v<StaticBody, T>) _pushBody(B, m_static_bodies);
    else if constexpr (std::is_base_of_v<RigidBody, T>)  _pushBody(B, m_rigid_bodies);
    else if constexpr (std::is_base_of_v<SoftBody, T>)   _pushBody(B, m_soft_bodies);

    std::cout << "m_static_bodies: " << m_static_bodies.size() << "\n";
    std::cout << "m_rigid_bodies:  " << m_rigid_bodies.size() << "\n";
    std::cout << "m_soft_bodies:   " << m_soft_bodies.size() << "\n\n";

    return B;
}


template <typename T, typename... Args>
T*
idk::phys::World::createConstraint( Args&&... args )
{
    auto *C = new T(std::forward<Args>(args)...);
    C->m_id = m_constraints.create(C);
    return C;
}




template <typename T, typename U>
void
idk::phys::World::_find_collisions( std::vector<T> &A, std::vector<U> &B )
{
    CollisionInfo info;

    for (int i=0; i<A.size(); i++)
    {
        int j0 = 0;

        if constexpr (std::is_same_v<T, U>)
        {
            j0 = i+1;
        }

        for (int j=j0; j<B.size(); j++)
        {
            if (A[i]->collides(B[j], &info))
            {
                // if (A[i] != info.A.body) std::swap(info.A, info.B);
                // A[i]->onCollisionEnter(info);

                // if (B[j] != info.A.body) std::swap(info.A, info.B);
                // B[j]->onCollisionEnter(info);

                m_collision_list.push_back(info);
            }
        }
    }
}
