#include "staticbody.hpp"
#include "../world.hpp"


idk::phys::StaticBody::StaticBody( World &world, const glm::vec3 &pos, Shape *shape )
:   Body(world, pos, shape)
{

}

