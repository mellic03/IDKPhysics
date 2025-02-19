#include "collision.hpp"

#include "../query.hpp"
#include <IDKPhysics/body/rigidbody.hpp>

#include <libidk/idk_math.hpp>
#include <IDKGraphics/terrain/terrain.hpp>


using namespace idk::phys;



bool
detectCollisionAABBAABB( Shape &A, Shape &B, CollisionInfo &info )
{
    return false;
}



bool
detectCollisionAABBSphere( Shape &A, Shape &B, CollisionInfo &info )
{
    auto &aabb = A.body->state;
    auto &sp   = B.body->state;

    glm::vec3 clamped = glm::clamp(sp.pos, aabb.pos-0.5f*A.extents, aabb.pos+0.5f*A.extents);

    // ren.drawSphere

    float distSq = glm::distance2(sp.pos, clamped);
    float dist = glm::sqrt(distSq);

    if (dist > B.radius)
    {
        return false;
    }

    info = {
        .A = A.body,
        .B = B.body,
        .contact = clamped,
        .normal  = -glm::normalize(sp.pos - clamped),
        .penetration = dist - B.radius
    };

    auto N = info.normal;
    // std::cout << dist << "\n";
    // std::cout << A.extents.x << ", " << A.extents.y << ", " << A.extents.z << "\n";
    // std::cout << B.radius << ", " << sp.pos.y << ", " << clamped.y << "\n";
    // std::cout << "\n";

    if (N != N)
    {
        // std::cout << "[detectCollisionAABBSphere] Bruh\n";
        exit(1);
    }

    return true;
}


bool
detectCollisionHeightmapSphere( Shape &hmap, Shape &sp, CollisionInfo &info )
{
    // LOG_INFO() << "[detectCollisionHeightmapSphere] " << int(hmap.type) << ", " << int(sp.type);
    
    auto &hmState = hmap.body->state;
    auto &sphere = sp.body->state;

    float sphere_bottom = sphere.pos.y - sp.radius;
    float terrain_y     = idk::TerrainRenderer::heightQuery(sphere.pos.x, sphere.pos.z);

    float overlap = terrain_y - sphere_bottom;

    if (overlap < 0.0f)
    {
        return false;
    }

    // Collision normal
    glm::vec3 N = idk::TerrainRenderer::slopeQuery(sphere.pos.x, sphere.pos.z);

    info = {
        .A           = hmap.body,
        .B           = sp.body,
        .normal      = -N,
        .penetration = overlap
    };

    return true;
}



bool
detectCollisionOBBSphere( Shape &obb, Shape &sp, CollisionInfo &info )
{
    return false;
    // const auto &obbState = obb.body->state;
    // const auto &spState  = sp.body->state;

    // glm::vec3 sphere_world  = spState.pos;
    // glm::vec3 sphere_local  = glm::inverse(obbState.T) * glm::vec4(sphere_world, 1.0f);
    // glm::vec3 nearest_local = glm::clamp(sphere_local, -0.5f*obb.extents, +0.5f*obb.extents);
    // glm::vec3 nearest_world = obbState.T * glm::vec4(nearest_local, 1.0f);

    // float distSq = glm::distance2(sphere_world, nearest_world);
    // float radius = sp.radius;

    // if (distSq > radius)
    // {
    //     return false;
    // }

    // auto  contact = nearest_world;
    // auto  disp    = sphere_world - nearest_world;
    // auto  dir     = glm::normalize(disp);
    // float overlap = glm::length(disp) - B.radius;


    // info = {
    //     .A = hmap.body,
    //     .B = sp.body,
    //     .contact     = contact,
    //     .normal      = -N,
    //     .penetration = -overlap*N
    // };

    // info.contactPoint = contact;

    // info.A = {
    //     .body   = A.body,
    //     .normal = dir
    // };

    // info.B = {
    //     .body   = B.body,
    //     .normal = -dir
    // };

    // return true;
}



bool
detectCollisionSphereSphere( Shape &sp1, Shape &sp2, CollisionInfo &info )
{
    auto &A = sp1.body->state;
    auto &B = sp2.body->state;

    float r1     = sp1.radius;
    float r2     = sp2.radius;
    float distSq = glm::distance2(A.pos, B.pos);

    if (distSq > (r1+r2)*(r1+r2))
    {
        return false;
    }

    // std::cout << "[detectCollisionSphereSphere]\n";

    auto  dir  = glm::normalize(A.pos-B.pos);
    float dist = glm::sqrt(distSq);

    auto contact = 0.5f * (A.pos - r1*dir)
                 + 0.5f * (B.pos + r2*dir);

    info = {
        .A = sp1.body,
        .B = sp2.body,
        .contact = contact,
        .normal = dir,
        .penetration = (r1+r2) - dist
    };

    return true;
}
