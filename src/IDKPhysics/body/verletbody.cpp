// #include "verletbody.hpp"
// #include <IDKPhysics/shape/sphere.hpp>
// #include <IDKPhysics/world.hpp>
// #include <libidk/idk_math.hpp>
// #include <IDKGraphics/terrain/terrain.hpp>


// idk::phys::VerletBody::VerletBody( World &world, const glm::vec3 &pos )
// :   RigidBody(world, pos, new ShapeSphere())
// {
//     mPrevBody = nullptr;
//     mNextBody = nullptr;
//     mCurrPos = pos;
//     mPrevPos = pos;
// }



// void
// idk::phys::VerletBody::integrate( KinematicState st, float dt )
// {
//     const auto &conf = m_world.config;

//     float K = 0.01f;

//     if (m_static == false)
//     {
//         // float h = idk::TerrainRenderer::heightQuery(mCurrPos.x, mCurrPos.z);

//         // float y_low = mCurrPos.y - shape->extents.x;
//         // float overlap = y_low - h;

//         // if (overlap < 0.0f)
//         // {
//         //     glm::vec3 N = idk::TerrainRenderer::slopeQuery(mCurrPos.x, mCurrPos.z);
//         //     mCurrPos += N*-overlap;
//         // }

//         glm::vec3 tmp = mCurrPos;
//         mCurrPos = (2.0f-K)*mCurrPos - (1.0f-K)*mPrevPos + dt*dt*(conf.gravity / st.invMass);
//         mPrevPos = tmp;
//     }

//     st.pos = mCurrPos;
//     // m_curr_state.pos = mCurrPos;

//     updateTransforms();
// }


