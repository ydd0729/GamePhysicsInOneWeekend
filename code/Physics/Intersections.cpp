#include "Intersections.h"
#include "GJK.h"

bool Intersect(Body* body_a, Body* body_b, contact_t& contact)
{
    contact.bodyA = body_a;
    contact.bodyB = body_b;
    
    const Vec3 ab = body_b->m_position - body_a->m_position;
    
    contact.normal = ab;
    contact.normal.Normalize();

    const auto sphere_a = dynamic_cast<const ShapeSphere*>(body_a->m_shape);
    const auto sphere_b = dynamic_cast<const ShapeSphere*>(body_b->m_shape);
    
    contact.ptOnA_WorldSpace = body_a->m_position + contact.normal * sphere_a->m_radius;
    contact.ptOnB_WorldSpace = body_b->m_position - contact.normal * sphere_b->m_radius;
    
    const float radius_ab = sphere_a->m_radius + sphere_b->m_radius;
    const float length_square = ab.GetLengthSqr();
    
    return length_square <= radius_ab * radius_ab;
}

bool Intersect(Body* bodyA, Body* bodyB, const float dt, contact_t& contact)
{
    // TODO: Add Code

    return false;
}
