#include "Scene.h"

#include <iostream>

#include "Physics/Contact.h"
#include "Physics/Intersections.h"
#include "Physics/Broadphase.h"

Scene::~Scene()
{
    for (int i = 0; i < m_bodies.size(); i++)
    {
        delete m_bodies[i].m_shape;
    }
    m_bodies.clear();
}

void Scene::Reset()
{
    for (int i = 0; i < m_bodies.size(); i++)
    {
        delete m_bodies[i].m_shape;
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

bool Intersect(Body* bodyA, Body* bodyB)
{
    const Vec3 ab = bodyB->m_position - bodyA->m_position;
    const auto sphere_a = dynamic_cast<const ShapeSphere*>(bodyA->m_shape);
    const auto sphere_b = dynamic_cast<const ShapeSphere*>(bodyB->m_shape);
    const float radius_ab = sphere_a->m_radius + sphere_b->m_radius;
    const float length_square = ab.GetLengthSqr();
    if (length_square <= radius_ab * radius_ab)
    {
        return true;
    }
    return false;
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
            
            if (Intersect(bodyA, bodyB))
            {
                bodyA->m_linearVelocity.Zero();
                bodyB->m_linearVelocity.Zero();
            }
        }
    }

    for (int i = 0; i < m_bodies.size(); i++)
    {
        // Position update
        m_bodies[i].m_position += m_bodies[i].m_linearVelocity * dt_sec;
    }
}
