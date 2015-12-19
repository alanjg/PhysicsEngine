#ifndef CONSTANTS_H
#define CONSTANTS_H

#include "Vector3.h"

namespace physics
{

    const float PENETRATION_EPSILON = 0.00001f;
    const float COLLISION_EPSILON = 0.01f;
    const float TIME_EPSILON = 0.0000001f;

	const float RESTING_CONTACT_EPSILON = 0.01f;

	const bool OUTPUT_DEBUG = false;
} // namespace physics

#endif