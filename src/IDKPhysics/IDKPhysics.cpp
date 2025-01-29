
#include "IDKPhysics.hpp"
#include <libidk/idk_random.hpp>


float
idk::phys::approxAirDensity( float altitude, float t )
{
    // NASA approximation: https://www.grc.nasa.gov/www/k-12/airplane/atmosmet.html
    float p = 2.488 * std::pow((t + 273.1) / 216.6, -11.388);
    return p / (0.2869 * (t + 273.1));
}

float
idk::phys::approxAirDensity( float altitude )
{
    float temperature = -131.21 + 0.00299*altitude;
    return approxAirDensity(altitude, temperature);
}




// void
// idk::phys::createRope( World &world, RigidBody *root )
// {
//     auto *ree = world.createBody<RigidBody>(glm::vec3(0, 16, 0));
//           ree->setMass(1.0f);
//           ree->setDrag(0.001f);
//           ree->m_static = true;

//     world.addConstraint(new SpringConstraint(root, ree, glm::vec3(0, -4, -4)));

//     glm::vec3   pos  = ree->state.pos + glm::vec3(0.25, -1, 0);
//     VerletBody *tail = ree;

//     for (int i=0; i<16; i++)
//     {
//         auto *next = world.createBody<VerletBody>(pos);
//               next->setMass(2.5f);
//               next->setDrag(0.001f);

//         world.addConstraint(new VerletConstraint(tail, next, 2.5f));
    
//         pos += glm::vec3(0.25, -1, 0);
//         tail = next;
//     }


//     tail->setMass(8.0f);
// }


void
idk::phys::createRope( World &world, RigidBody *root )
{
    auto *ree = world.createBody<VerletBody>(glm::vec3(0, 16, 0));
          ree->setMass(1.0f);
          ree->setDrag(0.001f);
          ree->m_static = true;

    world.createConstraint<GlueConstraint>(root, ree, glm::vec3(0, -4, -4));

    glm::vec3   pos  = ree->state.pos + glm::vec3(0.25, -1, 0);
    VerletBody *tail = ree;

    for (int i=0; i<16; i++)
    {
        auto *next = world.createBody<VerletBody>(pos);
              next->setMass(2.5f);
              next->setDrag(0.001f);

        world.createConstraint<VerletConstraint>(tail, next, 2.5f);
    
        pos += glm::vec3(0.25, -1, 0);
        tail = next;
    }

    tail->setMass(8.0f);
}

