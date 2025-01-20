#include "./common.hpp"
#include <vector>

#include "body/body.hpp"
#include "body/rigidbody.hpp"
#include "body/staticbody.hpp"



class idk::phys::WorldConfig
{
public:
    float drag_coef   = 1.0f;
    float fluid_density = 2.0f;

    glm::vec3 gravity = glm::vec3(0.0, -0.0, 0.0);
};



class idk::phys::World
{
private:
    float m_accum;
    float m_timestep;

    std::vector<Body*>       m_bodies;
    std::vector<StaticBody*> m_static_bodies;
    std::vector<RigidBody*>  m_rigid_bodies;

public:
    WorldConfig config;

          World( float timestep=1.0/30.0 );
         ~World();
    void  update( float dt );

    const auto &rigidBodies()  const { return m_rigid_bodies;  };
    const auto &staticBodies() const { return m_static_bodies; };

    template <typename body_type, typename... Args>
    body_type *createRigidBody( Args&&... args )
    {
        auto *obj = new body_type(*this, std::forward<Args>(args)...);
        m_rigid_bodies.push_back(dynamic_cast<RigidBody*>(obj));
        m_bodies.push_back(dynamic_cast<Body*>(obj));
        return obj;
    }

    template <typename body_type, typename... Args>
    body_type *createStaticBody( Args&&... args )
    {
        auto *obj = new body_type(*this, std::forward<Args>(args)...);
        m_static_bodies.push_back(dynamic_cast<StaticBody*>(obj));
        // m_bodies.push_back(dynamic_cast<Body*>(obj));
        return obj;
    }
};

