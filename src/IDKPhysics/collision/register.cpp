#include "collision.hpp"
#include <IDKPhysics/body/rigidbody.hpp>
#include <libidk/idk_log.hpp>
#include <unordered_map>

using namespace idk::phys;

using detect_type = std::function<bool(Shape&, Shape&, CollisionInfo&)>;
static std::unordered_map<uint32_t, detect_type> methodRegistry;

using resolve_type = std::function<bool(CollisionInfo&)>;
static std::unordered_map<uint32_t, resolve_type> methodRegistry2;



bool detectCollisionAABBAABB        ( Shape&, Shape&, CollisionInfo& );
bool detectCollisionAABBHeightmap   ( Shape&, Shape&, CollisionInfo& );
bool detectCollisionAABBOBB         ( Shape&, Shape&, CollisionInfo& );
bool detectCollisionAABBSphere      ( Shape&, Shape&, CollisionInfo& );
bool detectCollisionHeightmapOBB    ( Shape&, Shape&, CollisionInfo& );
bool detectCollisionHeightmapSphere ( Shape&, Shape&, CollisionInfo& );
bool detectCollisionOBBOBB          ( Shape&, Shape&, CollisionInfo& );
bool detectCollisionOBBSphere       ( Shape&, Shape&, CollisionInfo& );
bool detectCollisionSphereSphere    ( Shape&, Shape&, CollisionInfo& );


bool resolveCollisionDefault         ( CollisionInfo& );
bool resolveCollisionAABBAABB        ( CollisionInfo& );
bool resolveCollisionAABBHeightmap   ( CollisionInfo& );
bool resolveCollisionAABBOBB         ( CollisionInfo& );
bool resolveCollisionAABBSphere      ( CollisionInfo& );
bool resolveCollisionHeightmapOBB    ( CollisionInfo& );
bool resolveCollisionHeightmapSphere ( CollisionInfo& );
bool resolveCollisionOBBOBB          ( CollisionInfo& );
bool resolveCollisionOBBSphere       ( CollisionInfo& );
bool resolveCollisionSphereSphere    ( CollisionInfo& );


void
idk::phys::registerMethods()
{
    // methodRegistry[SHAPE_AABB      | SHAPE_AABB]   = detectCollisionAABBAABB;
    methodRegistry[SHAPE_AABB      | SHAPE_SPHERE] = detectCollisionAABBSphere;
    methodRegistry[SHAPE_OBB       | SHAPE_SPHERE] = detectCollisionOBBSphere;
    methodRegistry[SHAPE_SPHERE    | SHAPE_SPHERE] = detectCollisionSphereSphere;
    // methodRegistry[SHAPE_HEIGHTMAP | SHAPE_OBB]    = detectCollisionHeightmapOBB;
    methodRegistry[SHAPE_HEIGHTMAP | SHAPE_SPHERE] = detectCollisionHeightmapSphere;

    // methodRegistry2[SHAPE_AABB      | SHAPE_AABB]   = resolveCollisionAABBAABB;
    methodRegistry2[SHAPE_AABB      | SHAPE_SPHERE] = resolveCollisionAABBSphere;
    // methodRegistry2[SHAPE_OBB       | SHAPE_SPHERE] = resolveCollisionOBBSphere;
    // methodRegistry2[SHAPE_SPHERE    | SHAPE_SPHERE] = resolveCollisionSphereSphere;
    // methodRegistry2[SHAPE_HEIGHTMAP | SHAPE_OBB]    = resolveCollisionHeightmapOBB;
    methodRegistry2[SHAPE_HEIGHTMAP | SHAPE_SPHERE] = resolveCollisionHeightmapSphere;

}




bool
idk::phys::detectCollisionShapeShape( Shape *A, Shape *B, CollisionInfo &info )
{
    uint32_t key = A->type | B->type;

    if (methodRegistry.contains(key) == false)
    {
        LOG_WARN() << "[phys::detectCollisionShapeShape] Attempted to call unregistered method "
                   << "on shapes with types " << A->type << " and " << B->type;

        return false;
    }

    if (A->type > B->type)
    {
        std::swap(A, B);
    }

    return methodRegistry[key](*A, *B, info);
}



bool
idk::phys::resolveCollisionShapeShape( CollisionInfo &info )
{
    // if (info.A->shape.type > info.B->shape.type)
    // {
    //     std::swap(info.A, info.B);
    // }

    Body *A = info.A;
    Body *B = info.B;

    uint32_t key = A->shape.type | B->shape.type;

    if (methodRegistry2.contains(key) == false)
    {
        return resolveCollisionDefault(info);
    }

    return methodRegistry2[key](info);
}
