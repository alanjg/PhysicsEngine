#ifndef PLANE_H
#define PLANE_H
#include "Vector3.h"

namespace physics {

class Plane3f
{
public:
	Plane3f() {}
	Plane3f(const Vector3f& point,const Vector3f& normal);
	bool IsBehind(const Vector3f& point) const;
	float Distance(const Vector3f& point) const;
	void Set(float a,float b,float c,float d);
private:
	Vector3f normal;
	float distance;
};

class Plane3d
{
public:
	Plane3d() {}
	Plane3d(const Vector3d& point,const Vector3d& normal);
	bool IsBehind(const Vector3d& point) const;
	double Distance(const Vector3d& point) const;
	void Set(double a,double b,double c,double d);
private:
	Vector3d normal;
	double distance;
};

typedef Plane3f Plane;

} //namespace

#endif