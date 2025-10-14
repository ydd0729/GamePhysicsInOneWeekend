//
//  Contact.cpp
//
#include "Contact.h"

void ResolveContact(const contact_t& contact)
{
    Body* body_a = contact.bodyA;
    Body* body_b = contact.bodyB;

    const float inv_mass_a = body_a->m_invMass;
    const float inv_mass_b = body_b->m_invMass;

    // Calculate the collision impulse
    const Vec3 &n = contact.normal;
    const Vec3 vab = body_a->m_linearVelocity - body_b->m_linearVelocity;
    const float impulse = -2.0f * vab.Dot(n) / (inv_mass_a + inv_mass_b);
    const Vec3 impulse_vector = n * impulse;

    body_a->ApplyImpulseLinear(impulse_vector);
    body_b->ApplyImpulseLinear(impulse_vector * -1.0f);

    // Let’s also move our colliding objects to just outside of each other
    // 移动的比例与质量成反比，即保持两个物体的总重心不变
    const float t_a = body_a->m_invMass / (body_a->m_invMass + body_b->m_invMass);
    const float t_b = body_b->m_invMass / (body_a->m_invMass + body_b->m_invMass);
    
    const Vec3 ds = contact.ptOnB_WorldSpace - contact.ptOnA_WorldSpace;
    
    body_a->m_position += ds * t_a;
    body_b->m_position -= ds * t_b;
}
