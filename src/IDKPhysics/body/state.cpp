#include "state.hpp"
#include "../shape/sphere.hpp"
#include "../shape/OBB.hpp"
#include <libidk/idk_glm.hpp>

using namespace idk::phys;


KinematicState::KinematicState( const glm::vec3 &p, const glm::quat &r )
:   pos(p),
    rot(r),
    T(1.0f)
{

}


KinematicState
KinematicState::mix( const KinematicState &A, const KinematicState &B, float alpha )
{
    KinematicState AB(glm::vec3(0.0f));

    AB.pos = glm::mix(A.pos, B.pos, alpha);
    AB.rot = glm::slerp(A.rot, B.rot, alpha);

    auto T = glm::translate(glm::mat4(1.0f), AB.pos);
    auto R = glm::mat4_cast(AB.rot);
    AB.T = T * R;

    return AB;
}




void
TransformState::_update()
{
    T3    = glm::mat3(T);
    invT  = glm::inverse(T);
    invT3 = glm::inverse(T3);
}


void
TransformState::update( const glm::vec3 &pos, const glm::quat &rot )
{
    auto Tlocal = glm::translate(glm::mat4(1.0f), pos);
    auto Rlocal = glm::mat4_cast(rot);

    T = Tlocal * Rlocal;
    _update();
}


void
TransformState::update( const TransformState &parent, const glm::vec3 &pos,
                        const glm::quat &rot )
{
    auto Tlocal = glm::translate(glm::mat4(1.0f), pos);
    auto Rlocal = glm::mat4_cast(rot);

    T = parent.T * Tlocal * Rlocal;
    _update();
}


void
TransformState::update( const TransformState &parent, const TransformState &other )
{
    T = parent.T * other.T;
    _update();
}
