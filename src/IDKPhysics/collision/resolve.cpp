#include "collision.hpp"
#include <IDKPhysics/body/rigidbody.hpp>
#include <IDKPhysics/body/staticbody.hpp>
#include <libidk/idk_math.hpp>
#include <libidk/idk_log.hpp>


using namespace idk::phys;




bool resolveCollisionDefault( CollisionInfo &info )
{
    // LOG_INFO() << "[resolveCollisionDefault] "
    //            << int(info.A->shape.type) << ", " << int(info.B->shape.type);

    auto &A = info.A->state;
    auto &B = info.B->state;

    float mi1 = A.invMass;
    float mi2 = B.invMass;
    float e1  = A.restitution;
    float e2  = B.restitution;
    auto  &N  = info.normal;

    glm::vec3 vr = A.linear.vel - B.linear.vel;
    float     vj = -(1.0f + e1*e2) * glm::dot(vr, N);
    float     J  = vj / (mi1 + mi2);

    A.applyImpulseLinear(-mi1*J*N);
    B.applyImpulseLinear(+mi2*J*N);

    return true;
}



bool resolveCollisionAABBSphere( CollisionInfo &info )
{
    // LOG_INFO() << "[resolveCollisionAABBSphere] "
    //            << int(info.A->shape.type) << ", " << int(info.B->shape.type);

    auto &A = info.A->state;
    auto &B = info.B->state;

    float mi1 = A.invMass;
    float mi2 = B.invMass;
    float e1  = A.restitution;
    float e2  = B.restitution;
    auto  &N  = info.normal;

    glm::vec3 vr = A.linear.vel - B.linear.vel;
    float     vj = -(1.0f + e1*e2) * glm::dot(vr, N);
    float     J  = vj / (mi1 + mi2);

    // if (N != N)
    // {
    //     std::cout << "Bruh\n";
    //     exit(1);
    // }

    // A.applyImpulseLinear(-mi1*J*N);
    B.applyImpulseLinear(-mi2*J*N);
    B.applyTranslation(N*info.penetration);

    return true;
}



bool resolveCollisionHeightmapSphere( CollisionInfo &info )
{
    // std::cout << "[resolveCollisionHeightmapSphere] " << int(info.A->shape.type) << ", " << int(info.B->shape.type) << "\n";
    auto &A = info.A->state;
    auto &B = info.B->state;

    float mi1 = A.invMass;
    float mi2 = B.invMass;
    float e1  = A.restitution;
    float e2  = B.restitution;
    auto  &N  = info.normal;

    glm::vec3 vr = A.linear.vel - B.linear.vel;
    float     vj = -(1.0f + e1*e2) * glm::dot(vr, N);
    float     J  = vj / (mi1 + mi2);

    // A.applyImpulseLinear(+mi1*J*N);
    B.applyImpulseLinear(-mi2*J*N);
    B.applyTranslation(-N*info.penetration);
    // B.applyTranslation(-info.penetration*N);


    return true;
}


