#pragma once

namespace idk::phys
{
    class CollisionInfo;
    class CollisionList;
    class Body;
}



class idk::phys::CollisionList
{
private:


public:
    bool contains( Body*, Body* );
    void insert( const CollisionInfo& );
    void resolve();

};
