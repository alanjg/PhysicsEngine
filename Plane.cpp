#include "Plane.h"

namespace physics
{

Plane3f::Plane3f(const Vector3f& point,const Vector3f& vnormal):
	normal(vnormal)
{
	normal.Normalize();
	distance = -normal.Dot(point);
}

void Plane3f::Set(float a,float b,float c,float d)
{
	normal[0] = a;
	normal[1] = b;
	normal[2] = c;
	distance = d;
}

bool Plane3f::IsBehind(const Vector3f& point) const
{
	return Distance(point) < 0.0f;
}

float Plane3f::Distance(const Vector3f& point) const
{
	return point.Dot(normal) + distance;
}

Plane3d::Plane3d(const Vector3d& point,const Vector3d& vnormal):
	normal(vnormal)
{
	normal.Normalize();
	distance = -normal.Dot(point);
}

void Plane3d::Set(double a,double b,double c,double d)
{
	normal[0] = a;
	normal[1] = b;
	normal[2] = c;
	distance = d;
}

bool Plane3d::IsBehind(const Vector3d& point) const
{
	return Distance(point) < 0.0;
}

double Plane3d::Distance(const Vector3d& point) const
{
	return point.Dot(normal) + distance;
}
}