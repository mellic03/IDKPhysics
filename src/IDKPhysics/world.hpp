#include "./common.hpp"
#include "body/body.hpp"
#include "body/state.hpp"

#include "body-manager.hpp"

// #include <vector>
// #include <iostream>



class idk::phys::WorldConfig
{
public:
    WorldConfig();
    int   tickrate;
    int   substeps;
    float fluid_density;
    glm::vec3 gravity;

    float substepFactor() { return 1.0f / float(substeps); }
};



class idk::phys::World: public BodyManager
{
private:
    float m_accum;

    std::vector<std::deque<KinematicState>> m_history;
    // phys::BodyManager m_bodymanager;
    phys::CollisionList m_collisions;

    void _find_collisions( float dt );


public:
    WorldConfig config;

          World( uint32_t maxBodies = 1024 );
         ~World();

    void  update( float dt );
    bool raycast( const glm::vec3 &ro, const glm::vec3 &rd, glm::vec3 *hit, glm::vec3 *N,
                  Body **B );


    template <typename body_type, typename... Args>
    body_type *createBody( const glm::vec3 &pos, Args&&... args );


};




template <typename T, typename... Args>
T*
idk::phys::World::createBody( const glm::vec3 &pos, Args&&... args )
{
    // std::vector<KinematicState> *states;
    // std::vector<T*>             *T_bodies;

    if constexpr (std::is_base_of_v<StaticBody, T>)
    {
        auto states   = &m_static_states;
        auto T_bodies = &m_static_bodies;
        int bodyID  = m_bodyIDs.create();
        int stateID = states->size();
        states->push_back(KinematicState(pos));

        T *B = new T(*this, (*states)[stateID], std::forward<Args>(args)...);
        B->m_id = bodyID;
        B->state.updateTransforms();

        T_bodies->push_back(B);
        m_bodies.push_back(B);

        return B;
    }
    else if constexpr (std::is_base_of_v<RigidBody, T>)
    {
        auto states   = &m_rigid_states;
        auto T_bodies = &m_rigid_bodies;
        int bodyID  = m_bodyIDs.create();
        int stateID = states->size();
        states->push_back(KinematicState(pos));

        T *B = new T(*this, (*states)[stateID], std::forward<Args>(args)...);
        B->m_id = bodyID;
        B->state.updateTransforms();

        T_bodies->push_back(B);
        m_bodies.push_back(B);

        return B;
    }
    // else if constexpr (std::is_base_of_v<SoftBody, T>)
    // {
    //     // auto states   = &m_soft_states;
    //     auto T_bodies = &m_soft_bodies;
    //     int bodyID  = m_bodyIDs.create();
    //     int stateID = states->size();
    //     states->push_back(KinematicState(pos));

    //     T *B = new T(*this, (*states)[stateID], std::forward<Args>(args)...);
    //     B->m_id = bodyID;
    //     B->state.updateTransforms();

    //     T_bodies->push_back(B);
    //     m_bodies.push_back(B);

    //     return B;
    // }

    return nullptr;
}





// template <typename T, typename U>
// void
// idk::phys::World::_find_collisions( std::vector<T> &A, std::vector<U> &B )
// {
//     CollisionInfo info;

//     for (int i=0; i<A.size(); i++)
//     {
//         int j0 = 0;

//         if constexpr (std::is_same_v<T, U>)
//         {
//             j0 = i+1;
//         }

//         for (int j=j0; j<B.size(); j++)
//         {
//             info.A.body = A[i];
//             info.B.body = B[j];

//             if (A[i]->shape->collides(B[j]->shape, &info))
//             {
//                 m_collision_list.push_back(info);
//             }
//         }
//     }
// }
