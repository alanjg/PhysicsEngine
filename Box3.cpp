#include "Box3.h"
#include <iostream>
#include "Frustum.h"
#include "Vector2.h"
#include <algorithm>

namespace physics {


//retvalue:
//0 - no intersection
//1 - intersection
//2 - f contains *this
int Box3f::Contains(const Frustum3f& f) const
{
    Vector2f x(vmin[0],vmax[0]);
    Vector2f y(vmin[1],vmax[1]);
    Vector2f z(vmin[2],vmax[2]);
    for(int p=0;p<6;p++)
    {
        int count=0;
        for(int i=0;i<2;i++)
            for(int j=0;j<2;j++)
                for(int k=0;k<2;k++)
					if(!f.planes[p].IsBehind(Vector3f(x[i],y[j],z[k])))
                        count++;
        if(count==0)
            return 0;

        //not contained in this plane
        if(count!=8)
            return 1;
    } 
    return 2;
}


bool Box3f::Intersects(const Frustum3f& f) const
{
    Vector2f x(vmin[0],vmax[0]);
    Vector2f y(vmin[1],vmax[1]);
    Vector2f z(vmin[2],vmax[2]);
    for(int p=0;p<6;p++)
    {
        for(int i=0;i<2;i++)
            for(int j=0;j<2;j++)
                for(int k=0;k<2;k++)
                    if(!f.planes[p].IsBehind(Vector3f(x[i],y[j],z[k])))
                        goto next_plane;

        return false;
next_plane:;
    } 
    return true;
}

bool Box3f::Intersect(const Box3f& bb) const
{
    for(int i=0;i<3;i++)
        if(vmin[i]>bb.vmax[i] || vmax[i]<bb.vmin[i])
            return false;
    return true;
}

bool Box3f::Contains(const Box3f& bb) const
{
    for(int i=0;i<3;i++)
        if(!(vmin[i] <= bb.vmin[i] && vmax[i] >= bb.vmax[i]))
            return false;
    return true;
}

bool Box3f::Intersect(const Vector3f& point) const
{
	return vmin[0]<=point[0] && point[0]<=vmax[0] && 
		vmin[1]<=point[1] && point[1]<=vmax[1] && 
		vmin[2]<=point[2] && point[2]<=vmax[2];
}

bool Box3f::Intersect(Vector3f a,Vector3f b,Vector3f c) const
{
    for(int i=0;i<3;i++)
    if( a[i] < vmin[i] && b[i] < vmin[i] && c[i] < vmin[i] ||
        a[i] > vmax[i] && b[i] > vmax[i] && c[i] > vmax[i]  )
        return false;

    //will generate false positives
    //TODO: fix later
    return true;
}

void Box3f::Extend(const Box3f& box)
{
	vmin[0] = std::min(vmin[0],box.vmin[0]);
	vmin[1] = std::min(vmin[1],box.vmin[1]);
	vmin[2] = std::min(vmin[2],box.vmin[2]);
	
	vmax[0] = std::max(vmax[0],box.vmax[0]);
	vmax[1] = std::max(vmax[1],box.vmax[1]);
	vmax[2] = std::max(vmax[2],box.vmax[2]);
}


std::ostream& operator<<(std::ostream& out,const Box3f& box)
{
    return out<<box.vmin<<" "<<box.vmax<<std::endl;
}

//retvalue:
//0 - no intersection
//1 - intersection
//2 - f contains *this
int Box3d::Contains(const Frustum3d& f) const
{
    Vector2d x(vmin[0],vmax[0]);
    Vector2d y(vmin[1],vmax[1]);
    Vector2d z(vmin[2],vmax[2]);
    for(int p=0;p<6;p++)
    {
        int count=0;
        for(int i=0;i<2;i++)
            for(int j=0;j<2;j++)
                for(int k=0;k<2;k++)
                    if(!f.planes[p].IsBehind(Vector3d(x[i],y[j],z[k])))
                        count++;
        if(count==0)
            return 0;

        //not contained in this plane
        if(count!=8)
            return 1;
    } 
    return 2;
}


bool Box3d::Intersects(const Frustum3d& f) const
{
    Vector2d x(vmin[0],vmax[0]);
    Vector2d y(vmin[1],vmax[1]);
    Vector2d z(vmin[2],vmax[2]);
    for(int p=0;p<6;p++)
    {
        for(int i=0;i<2;i++)
            for(int j=0;j<2;j++)
                for(int k=0;k<2;k++)
                    if(!f.planes[p].IsBehind(Vector3d(x[i],y[j],z[k])))
                        goto next_plane;

        return false;
next_plane:;
    } 
    return true;
}

bool Box3d::Intersect(const Box3d& bb) const
{
    for(int i=0;i<3;i++)
        if(vmin[i]>bb.vmax[i] || vmax[i]<bb.vmin[i])
            return false;
    return true;
}

bool Box3d::Contains(const Box3d& bb) const
{
    for(int i=0;i<3;i++)
        if(!(vmin[i] <= bb.vmin[i] && vmax[i] >= bb.vmax[i]))
            return false;
    return true;
}

bool Box3d::Intersect(const Vector3d& point) const
{
	return vmin[0]<=point[0] && point[0]<=vmax[0] && 
		vmin[1]<=point[1] && point[1]<=vmax[1] && 
		vmin[2]<=point[2] && point[2]<=vmax[2];
}

bool Box3d::Intersect(Vector3d a,Vector3d b,Vector3d c) const
{
    for(int i=0;i<3;i++)
    if( a[i] < vmin[i] && b[i] < vmin[i] && c[i] < vmin[i] ||
        a[i] > vmax[i] && b[i] > vmax[i] && c[i] > vmax[i]  )
        return false;

    //will generate false positives
    //TODO: fix later
    return true;
}

std::ostream& operator<<(std::ostream& out,const Box3d& box)
{
    return out<<box.vmin<<" "<<box.vmax<<std::endl;
}


} //namespace