#ifndef FRUSTUM_H
#define FRUSTUM_H

#include "Plane.h"

namespace physics {

struct Frustum3f
{
	Plane3f planes[6];
};

struct Frustum3d
{
	Plane3d planes[6];
};

typedef Frustum3f Frustum;

} //namespace

#endif