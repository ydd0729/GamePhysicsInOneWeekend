#include "Scene.h"

#include <iostream>

#include "Physics/Contact.h"
#include "Physics/Intersections.h"
#include "Physics/Broadphase.h"

Scene::~Scene()
{
    for (auto& m_bodie : m_bodies)
    {
        delete m_bodie.m_shape;
    }
    m_bodies.clear();
}

void Scene::Reset()
{
    for (const auto& m_bodie : m_bodies)
    {
        delete m_bodie.m_shape;
    }
    m_bodies.clear();

    Initialize();
}

void Scene::Initialize()
{
    Body body;
    body.m_position = Vec3(0, 0, 10);
    body.m_orientation = Quat(0, 0, 0, 1);
    body.m_invMass = 1.0f;
    body.m_shape = new ShapeSphere(1.0f);
    m_bodies.push_back(body);

    body.m_position = Vec3(0, 0, -1000);
    body.m_orientation = Quat(0, 0, 0, 1);
    body.m_invMass = 0.0f;
    body.m_shape = new ShapeSphere(1000.0f);
    m_bodies.push_back(body);
}

void Scene::Update(const float dt_sec)
{
    for (int i = 0; i < m_bodies.size(); i++)
    {
        Body* body = &m_bodies[i];

        // Gravity needs to be an impulse
        // I = dp , F = dp/ dt => dp = F * dt => I = F * dt
        // F = mgs
        float mass = 1.0f / body->m_invMass;
        Vec3 impulseGravity = Vec3(0, 0, -10) * mass * dt_sec;
        body->ApplyImpulseLinear(impulseGravity);
    }

    // Check for collisions with other bodies
    for (int i = 0; i < m_bodies.size(); i++)
    {
        for (int j = i + 1; j < m_bodies.size(); j++)
        {
            Body* bodyA = &m_bodies[i];
            Body* bodyB = &m_bodies[j];

            // Skip body pairs with infinite mass
            if (0.0f == bodyA->m_invMass && 0.0f == bodyB->m_invMass)
            {
                continue;
            }

            contact_t contact;
            if (Intersect(bodyA, bodyB, contact))
            {
                ResolveContact(contact);
            }
        }
    }

    for (auto& m_bodie : m_bodies)
    {
        // Position update
        m_bodie.m_position += m_bodie.m_linearVelocity * dt_sec;
    }
}
