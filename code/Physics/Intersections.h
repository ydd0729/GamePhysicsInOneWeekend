//
//	Intersections.h
//
#pragma once
#include "Contact.h"

bool Intersect( Body * body_a, Body * body_b, contact_t & contact );
bool Intersect( Body * bodyA, Body * bodyB, const float dt, contact_t & contact );
