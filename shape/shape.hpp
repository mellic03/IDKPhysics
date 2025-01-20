#pragma once
#include "../common.hpp"




class idk::phys::Shape
{
public:
    enum class Type
    {
        NONE = 0,
        SPHERE,
        CUBOID
    };

protected:
    const Type m_type;

public:
    glm::vec3 extents = glm::vec3(1.0f);

    Shape( Type t ): m_type(t) {  };

    Type getType() const { return m_type; }
    virtual bool intersects( Shape*) = 0;

};

