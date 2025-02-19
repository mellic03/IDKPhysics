#pragma once

#include "body/state.hpp"
#include "collision/collision.hpp"
#include "constraint/constraint.hpp"

#include <libidk/idk_allocator.hpp>
#include <libidk/idk_wallocator2.hpp>

#include <cstdint>
#include <cstddef>
#include <map>



namespace idk::phys
{
    class BodyManager;
}


class idk::phys::BodyManager
{
protected:
    const int MAX_BODIES;

    idk::Allocator<int>            m_bodyIDs;
    std::vector<Body*>             m_bodies;

    std::vector<StaticBody*>       m_static_bodies;
    std::vector<RigidBody*>        m_rigid_bodies;
    std::vector<SoftBody*>         m_soft_bodies;

    std::vector<KinematicState>    m_states;
    std::vector<KinematicState>    m_static_states;
    std::vector<KinematicState>    m_rigid_states;
    // std::vector<KinematicState>    m_soft_states;

    idk::WAllocator2<Constraint>   m_constraints;

    // std::map<size_t, std::vector<KinematicState> m_states>;

    template <typename T>
    static size_t getkey()
    {
        return typeid(T).hash_code();
    };

    void _applyGForces( float dt, int substeps );
    void _integrateLinear( float dt, int substeps );
    void _integrateAngular( float dt, int substeps );
    void _integrate( float dt, int substeps );





public:

     BodyManager( size_t maxBodies = 1024 );
    ~BodyManager();

    void updateBodies( float dt, int tickrate, int substeps );

    const auto &rigidBodies()  const { return m_rigid_bodies;  };
    const auto &staticBodies() const { return m_static_bodies; };

    template <typename constraint_type, typename... Args>
    constraint_type *createConstraint( Args&&... );

    void deleteConstraint( int id ) { m_constraints.destroy(id); }
};





template <typename T, typename... Args>
T*
idk::phys::BodyManager::createConstraint( Args&&... args )
{
    auto *C = new T(std::forward<Args>(args)...);
    C->m_id = m_constraints.create(C);
    return C;
}

