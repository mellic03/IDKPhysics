#include "staticbody.hpp"
#include "../world.hpp"


idk::phys::StaticBody::StaticBody( World &world, const glm::vec3 &pos )
:   Body(world, pos)
{

}


idk::phys::StaticBody::~StaticBody()
{

}


