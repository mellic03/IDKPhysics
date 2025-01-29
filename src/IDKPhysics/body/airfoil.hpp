#pragma once
#include "rigidbody.hpp"

namespace idk::phys
{
    class Airfoil;
}


class idk::phys::Airfoil: public idk::phys::RigidBody
{
protected:
    float m_span;
    float m_chord;
    float m_area;
    float m_aspect_ratio;
    float m_flap_ratio;
    float m_min_aoa;
    float m_max_aoa;
    std::vector<glm::vec3> m_data;

    std::pair<float, float> _sampleLiftDragCoefficients( float aoa );

public:
    Airfoil( idk::phys::World&, const glm::vec3&, float span=3.8f, float chord=1.26f );

    virtual void update( float alpha ) override;
    virtual void integrate( float dt ) override;

    virtual glm::vec3 computeAeroForcesLinear() override;
};
