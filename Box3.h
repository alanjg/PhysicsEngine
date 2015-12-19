#ifndef BOX3_H
#define BOX3_H
#include "Vector3.h"
#include <iosfwd>
//bounding box class
namespace physics {

struct Frustum3f;

class Box3f
{
public:
	Box3f() {}
	Box3f(const Vector3f& _min,const Vector3f& _max): vmin(_min),vmax(_max) {}

	void SetMin(const Vector3f& _min) { vmin=_min; }
	void SetMin(float x,float y,float z) { vmin[0]=x; vmin[1]=y; vmin[2]=z; }
	void SetMax(const Vector3f& _max) { vmax=_max; }
	void SetMax(float x,float y,float z) { vmax[0]=x; vmax[1]=y; vmax[2]=z; }

	void SetBounds(const Vector3f& _min,const Vector3f& _max) { vmin=_min; vmax=_max; }
	void SetBounds(float xmin,float ymin,float zmin,float xmax,float ymax,float zmax) 
		{ vmin[0]=xmin; vmin[1]=ymin; vmin[2]=zmin; vmax[0]=xmax; vmax[1]=ymax; vmax[2]=zmax; }
	void GetBounds(Vector3f& _min,Vector3f& _max) { _min=vmin; _max=vmax; }
	void GetBounds(float &xmin,float &ymin,float &zmin,float &xmax,float &ymax,float &zmax) 
		{ xmin=vmin[0]; ymin=vmin[1]; zmin=vmin[2]; xmax=vmax[0]; ymax=vmax[1]; zmax=vmax[2]; }

	void Extend(const Box3f& box);

    const Vector3f &	GetMin() const { return vmin; }
    const Vector3f &	GetMax() const { return vmax; }
    Vector3f &GetMin() { return vmin; }
    Vector3f &GetMax() { return vmax; }

	Vector3f GetCenter() const { return (vmin+vmax)/2.0f; }
	
    //retvalue:
    //0 - no intersection
    //1 - intersection
    //2 - f contains *this
    int Contains(const Frustum3f& f) const;

    //plain intersection
    bool Intersects(const Frustum3f& f) const;

	//intersect with another box
	bool Intersect(const Box3f& bb) const;

    //true if *this contains bb
    bool Contains(const Box3f& bb) const;

	//intersect with a point
	bool Intersect(const Vector3f& point) const;

    //rough intersect with a triangle
    //WILL GENERATE FALSE POSITIVES
    bool Intersect(Vector3f a,Vector3f b,Vector3f c) const;
	
	friend std::ostream& operator<<(std::ostream& out,const Box3f& box);
private:
	Vector3f vmin,vmax;
};

struct Frustum3d;

class Box3d
{
public:
	Box3d() {}
	Box3d(const Vector3d& _min,const Vector3d& _max): vmin(_min),vmax(_max) {}

	void SetMin(const Vector3d& _min) { vmin=_min; }
	void SetMin(double x,double y,double z) { vmin[0]=x; vmin[1]=y; vmin[2]=z; }
	void SetMax(const Vector3d& _max) { vmax=_max; }
	void SetMax(double x,double y,double z) { vmax[0]=x; vmax[1]=y; vmax[2]=z; }

	void SetBounds(const Vector3d& _min,const Vector3d& _max) { vmin=_min; vmax=_max; }
	void SetBounds(double xmin,double ymin,double zmin,double xmax,double ymax,double zmax) 
		{ vmin[0]=xmin; vmin[1]=ymin; vmin[2]=zmin; vmax[0]=xmax; vmax[1]=ymax; vmax[2]=zmax; }
	void GetBounds(Vector3d& _min,Vector3d& _max) { _min=vmin; _max=vmax; }
	void GetBounds(double &xmin,double &ymin,double &zmin,double &xmax,double &ymax,double &zmax) 
		{ xmin=vmin[0]; ymin=vmin[1]; zmin=vmin[2]; xmax=vmax[0]; ymax=vmax[1]; zmax=vmax[2]; }

    const Vector3d &	GetMin() const { return vmin; }
    const Vector3d &	GetMax() const { return vmax; }
    Vector3d &GetMin() { return vmin; }
    Vector3d &GetMax() { return vmax; }

	Vector3d GetCenter() const { return (vmin+vmax)/2.0f; }
	
    //retvalue:
    //0 - no intersection
    //1 - intersection
    //2 - f contains *this
    int Contains(const Frustum3d& f) const;

    //plain intersection
    bool Intersects(const Frustum3d& f) const;

	//intersect with another box
	bool Intersect(const Box3d& bb) const;

    //true if *this contains bb
    bool Contains(const Box3d& bb) const;

	//intersect with a point
	bool Intersect(const Vector3d& point) const;

    //rough intersect with a triangle
    //WILL GENERATE FALSE POSITIVES
    bool Intersect(Vector3d a,Vector3d b,Vector3d c) const;
	
	friend std::ostream& operator<<(std::ostream& out,const Box3d& box);
private:
	Vector3d vmin,vmax;
};

typedef Box3f Box3;

}
#endif