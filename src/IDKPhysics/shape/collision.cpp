#include "collision.hpp"
#include "sphere.hpp"
#include "AABB.hpp"
#include "OBB.hpp"
#include "heightmap.hpp"
#include "../query.hpp"
#include "../body/staticbody.hpp"
#include "../body/rigidbody.hpp"

#include <IDKPhysics/body/body.hpp>
#include <libidk/idk_math.hpp>
#include <IDKGraphics/terrain/terrain.hpp>


// using namespace idk::phys;


bool
idk::phys::collisionTest( ShapeAABB &A, ShapeAABB &B, CollisionInfo *info )
{
    return false;
}


bool
idk::phys::collisionTest( ShapeAABB &A, ShapeHeightmap &B, CollisionInfo *info )
{
    return false;
}


bool
idk::phys::collisionTest( ShapeAABB&, ShapeOBB&, CollisionInfo* )
{
    return false;
}


bool
idk::phys::collisionTest( ShapeAABB &A, ShapeSphere &B, CollisionInfo *info )
{
    const auto &Aspace = A.body->state;
    const auto &Bspace = B.body->state;

    glm::mat4 &A_T = A.body->state.T;
    glm::mat4 &B_T = B.body->state.T;

    glm::vec3 sphere_world   = Bspace.T[0];
    glm::vec3 sphere_local   = glm::inverse(Aspace.T) * glm::vec4(sphere_world, 1.0f);
    glm::vec3 nearest_local = glm::clamp(sphere_local, -0.5f*A.extents, +0.5f*A.extents);
    glm::vec3 nearest_world = Aspace.T * glm::vec4(nearest_local, 1.0f);

    float distSq = glm::distance2(sphere_world, nearest_world);
    float radius = B.radius;

    if (distSq > radius)
    {
        return false;
    }

    auto  contact = nearest_world;
    auto  disp    = sphere_world - nearest_world;
    auto  dir     = glm::normalize(disp);
    float overlap = glm::length(disp) - B.radius;


    info->contactPoint = contact;

    info->A = {
        .body   = A.body,
        .normal = dir
    };

    info->B = {
        .body   = B.body,
        .normal = -dir
    };

    return true;
}


bool
idk::phys::collisionTest( ShapeHeightmap&, ShapeOBB&, CollisionInfo* )
{
    return false;
}


bool
idk::phys::collisionTest( ShapeHeightmap &A, ShapeSphere &B, CollisionInfo *info )
{
    auto *bodyA = dynamic_cast<StaticBody*>(A.body);
    auto *bodyB = dynamic_cast<RigidBody*>(B.body);
    if (!bodyA || !bodyB)
    {
        return false;
    }

    auto &pos = bodyB->state.pos;
    auto  hit = pos;
          hit.y = idk::TerrainRenderer::heightQuery(pos.x, pos.z);

    float y_low = pos.y - B.radius;
    float overlap = y_low - hit.y;

    if (overlap >= 0.0f)
    {
        return false;
    }

    // Collision normal
    glm::vec3 N = idk::TerrainRenderer::slopeQuery(pos.x, pos.z);
    if (N != N)
    {
        return false;
    }

    // A/B resitution, normally denoted as e
    float Ae = 1.0f;
    float Be = bodyB->getRestitution();

    float Am = 1.0f;
    float Bm = bodyB->getMass();
    float totalMass = Bm;

    // A/B velocity
    glm::vec3 Av = glm::vec3(0.0f);
    glm::vec3 Bv = bodyB->state.linear.vel;

    glm::vec3 vr = Bv - Av; // relative velocity
    float     vj = -(1.0f + Be) * glm::dot(vr, N);
    glm::vec3 impulse = -(vj / Bm) * N;
    glm::vec3 delta   = -overlap * N;


    info->contactPoint = hit;

    info->A = {
        .body    = bodyA,
        .normal  = -N,
        .penetration = -overlap * N
    };

    info->B = {
        .body    = bodyB,
        .normal  = N,
        .penetration = +overlap * N
    };

    return true;
}



bool
idk::phys::collisionTest( ShapeOBB&, ShapeOBB&, CollisionInfo* )
{
    return false;
}


bool
idk::phys::collisionTest( ShapeOBB&, ShapeSphere&, CollisionInfo* )
{
    return false;
}




bool
idk::phys::collisionTest( ShapeSphere &A, ShapeSphere &B, CollisionInfo *info )
{
    auto *bodyA = dynamic_cast<RigidBody*>(A.body);
    auto *bodyB = dynamic_cast<RigidBody*>(B.body);
    if (!bodyA || !bodyB)
    {
        return false;
    }

    const auto &Apos = A.body->state.pos;
    const auto &Bpos = B.body->state.pos;

    float Arad   = A.extents.x;
    float Brad   = B.extents.x;
    float distSq = glm::distance2(Apos, Bpos);

    if (distSq > (Arad+Brad)*(Arad+Brad))
    {
        return false;
    }

    if (info)
    {
        auto  dir  = glm::normalize(Bpos-Apos);
        float dist = A.radius + B.radius;

        info->contactPoint = 0.5f * (Apos + A.radius*dir)
                           + 0.5f * (Bpos - B.radius*dir);

        info->A = {
            .body    = bodyA,
            .normal  = dir
        };

        info->B = {
            .body    = bodyB,
            .normal  = dir
        };
    }

    return true;
}
