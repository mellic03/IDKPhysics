#include "collision-list.hpp"
#include "collision.hpp"
#include <IDKPhysics/body/body.hpp>

#include <map>

using namespace idk::phys;


using key_type = std::pair<int, int>;
static std::map<key_type, CollisionInfo> m_map;

static std::pair<int, int>
getKey( Body *A, Body *B )
{
    if (A->getID() < B->getID())
    {
        std::swap(A, B);
    }

    return std::make_pair(A->getID(), B->getID());
}


void
CollisionList::resolve()
{
    static std::vector<key_type> cull;

    for (auto &[key, info]: m_map)
    {
        if (phys::resolveCollisionShapeShape(info))
        {
            cull.push_back(key);
        }
    }

    for (auto &key: cull)
    {
        auto &info = m_map[key];
        info.A->onCollisionExit(info);
        info.B->onCollisionExit(info);
        m_map.erase(key);
    }

    cull.clear();
}


bool
CollisionList::contains( Body *A, Body *B )
{
    auto key = getKey(A, B);
    return m_map.contains(key);
}


void
CollisionList::insert( const CollisionInfo &info )
{
    auto key = getKey(info.A, info.B);

    if (m_map.contains(key) == false)
    {
        m_map[key] = info;
    }

    else
    {

    }
}
