#pragma once
#include "../common.hpp"
#include "body.hpp"


class idk::phys::StaticBody: public idk::phys::Body
{
private:
    friend class World;

    glm::vec3 m_pos;
    glm::quat m_rot;

public:
    StaticBody( idk::phys::World&, const glm::vec3& );
    ~StaticBody();
};
