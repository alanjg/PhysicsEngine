//-----------------------------------------------------------------------------
// Intersections.cpp
//
// Intersection tests for objects of various types
//-----------------------------------------------------------------------------


#include "Intersection.h"
#include "SphereBody.h"
#include "PolyMeshBody.h"
#include "Constants.h"
#include <utility>
#include <algorithm>

namespace physics
{

//-----------------------------------------------------------------------------
// Triangle-Triangle
//   "Faster Triangle-Triangle Intersection Tests",
//   Olivier Devillers, Philippe Guigue
//-----------------------------------------------------------------------------

static bool IsCrossingPlane( const Vector3& planeNormal, const Vector3& planePt, const Tri& tri2 )
{
	// Project the points of the second triangle onto the normal vector of the plane
	float tri2Min = std::min( planeNormal.Dot(tri2.p[0]), std::min( planeNormal.Dot(tri2.p[1]), planeNormal.Dot(tri2.p[2]) ) );
	float tri2Max = std::max( planeNormal.Dot(tri2.p[0]), std::max( planeNormal.Dot(tri2.p[1]), planeNormal.Dot(tri2.p[2]) ) );

	// Then see whether it crosses
	float tri1Pos = planeNormal.Dot(planePt);
	if( tri1Pos < tri2Min || tri1Pos > tri2Max )
		return false;
	// else
	return true;
}

static bool IsCrossingPlane( const Vector3& planeNormal, const Tri& tri1, const Tri& tri2 )
{
	// Project the points of the triangles onto the normal vector of the plane
	float tri1Min = std::min( planeNormal.Dot(tri1.p[0]), std::min( planeNormal.Dot(tri1.p[1]), planeNormal.Dot(tri1.p[2]) ) );
	float tri1Max = std::max( planeNormal.Dot(tri1.p[0]), std::max( planeNormal.Dot(tri1.p[1]), planeNormal.Dot(tri1.p[2]) ) );

	float tri2Min = std::min( planeNormal.Dot(tri2.p[0]), std::min( planeNormal.Dot(tri2.p[1]), planeNormal.Dot(tri2.p[2]) ) );
	float tri2Max = std::max( planeNormal.Dot(tri2.p[0]), std::max( planeNormal.Dot(tri2.p[1]), planeNormal.Dot(tri2.p[2]) ) );

	// See whether the triangles are on the same side or not
	if( tri1Max < tri2Min || tri2Max < tri1Min )
		return false;
	// else
	return true;
}

bool IsIntersecting( const Tri& tri1, const Tri& tri2 )
{
	Vector3 tri1Edges[3] = { tri1.p[1]-tri1.p[0], tri1.p[2]-tri1.p[1], tri1.p[0]-tri1.p[2] };
	Vector3 tri2Edges[3] = { tri2.p[1]-tri2.p[0], tri2.p[2]-tri2.p[1], tri2.p[0]-tri2.p[2] };
	Vector3 tri1Normal = tri1Edges[0].Cross(tri1Edges[1]);
	Vector3 tri2Normal = tri2Edges[0].Cross(tri2Edges[1]);

	// Check whether the containing planes cross
	if( !IsCrossingPlane(tri1Normal, tri1.p[0], tri2) )
		return false;

	// Test for coplanarity using the sine of the angle between the normals
	if( ( tri1Normal.Cross(tri2Normal) ).MagnitudeSquared() >=
		FLOAT_TOLERANCE  * ( tri1Normal.MagnitudeSquared() * tri2Normal.MagnitudeSquared() ) )
	{
		// not parallel

		if( !IsCrossingPlane(tri2Normal, tri2.p[0], tri1) )
			return false;

		// Check all possible separating planes, defined by one edge from each triangle
		for( int i = 0; i < 3; i++ )
		{
			for( int j = 0; j < 3; j++ )
			{
				if( !IsCrossingPlane(tri1Edges[j].Cross(tri2Edges[i]), tri1, tri2) )
					return false;
			}
		}
	}
	else  // parallel and coplanar
	{
		// Possible separating planes are defined by the normal and an edge
		for( int i = 0; i < 3; i++ )
		{
			if( !IsCrossingPlane(tri1Normal.Cross(tri1Edges[i]), tri1, tri2) ||
				!IsCrossingPlane(tri2Normal.Cross(tri2Edges[i]), tri1, tri2) )
				return false;
		}
	}

	return true;
}


//-----------------------------------------------------------------------------
// OBB-OBB
// This one's straight out of the Gamasutra article by Miguel Gomez, October '99
//-----------------------------------------------------------------------------

bool IsIntersecting( const BBox& box1_, const BBox& box2_, const Transform& mesh1Trans, const Transform& mesh2Trans, float f )
{
	BBox& box1 = const_cast<BBox&>(box1_);
	BBox& box2 = const_cast<BBox&>(box2_);
	Vector3 box1Pos = mesh1Trans.BodyToWorldSpace( box1.m_trans );
	Vector3 box2Pos = mesh2Trans.BodyToWorldSpace( box2.m_trans );
	Matrix33 box1Rot;
	Matrix33 box2Rot;
	box1Rot.MakeRotate( mesh1Trans.GetRotation().GetAngle(), mesh1Trans.GetRotation().GetAxis() );
	box1Rot = box1.m_rot * box1Rot.Transpose();

	box2Rot.MakeRotate( mesh2Trans.GetRotation().GetAngle(), mesh2Trans.GetRotation().GetAxis() );
	box2Rot = box2.m_rot * box2Rot.Transpose();
	

	// translation, in box1's frame
	Vector3 T( box1Rot * (box2Pos - box1Pos) );

	// box2's basis with respect to box1's local frame
	Matrix33 R( box1Rot[0].Dot(box2Rot[0]), box1Rot[0].Dot(box2Rot[1]), box1Rot[0].Dot(box2Rot[2]),
	            box1Rot[1].Dot(box2Rot[0]), box1Rot[1].Dot(box2Rot[1]), box1Rot[1].Dot(box2Rot[2]),
	            box1Rot[2].Dot(box2Rot[0]), box1Rot[2].Dot(box2Rot[1]), box1Rot[2].Dot(box2Rot[2]) );


	// ALGORITHM: Use the separating axis test for all 15 potential 
	// separating axes. If a separating axis could not be found, the two 
	// boxes overlap.

	box1.m_ext[0] += f; box1.m_ext[1] += f; box1.m_ext[2] += f;
	box2.m_ext[0] += f; box2.m_ext[1] += f; box2.m_ext[2] += f;

	// box1's basis vectors
	if( fabsf(T[0]) > box1.m_ext[0] +
	        box2.m_ext[0]*fabsf(R[0][0]) + box2.m_ext[1]*fabsf(R[0][1]) + box2.m_ext[2]*fabsf(R[0][2]) ) 
		return false;

	if( fabsf(T[1]) > box1.m_ext[1] +
	        box2.m_ext[0]*fabsf(R[1][0]) + box2.m_ext[1]*fabsf(R[1][1]) + box2.m_ext[2]*fabsf(R[1][2]) ) 
		return false;

	if( fabsf(T[2]) > box1.m_ext[2] +
	        box2.m_ext[0]*fabsf(R[2][0]) + box2.m_ext[1]*fabsf(R[2][1]) + box2.m_ext[2]*fabsf(R[2][2]) ) 
		return false;

	// box2's basis vectors
	if( fabsf(T[0]*R[0][0] + T[1]*R[1][0] + T[2]*R[2][0]) >
	        box1.m_ext[0]*fabsf(R[0][0]) + box1.m_ext[1]*fabsf(R[1][0]) + box1.m_ext[2]*fabsf(R[2][0]) +
	        box2.m_ext[0] )
		return false;

	if( fabsf(T[0]*R[0][1] + T[1]*R[1][1] + T[2]*R[2][1]) >
	        box1.m_ext[0]*fabsf(R[0][1]) + box1.m_ext[1]*fabsf(R[1][1]) + box1.m_ext[2]*fabsf(R[2][1]) +
	        box2.m_ext[1] )
		return false;

	if( fabsf(T[0]*R[0][2] + T[1]*R[1][2] + T[2]*R[2][2]) >
	        box1.m_ext[0]*fabsf(R[0][2]) + box1.m_ext[1]*fabsf(R[1][2]) + box1.m_ext[2]*fabsf(R[2][2]) +
	        box2.m_ext[2] )
		return false;

	// 9 cross products

	// L = A0 x b0
	if( fabsf(T[2]*R[1][0] - T[1]*R[2][0]) >
	        box1.m_ext[1]*fabsf(R[2][0]) + box1.m_ext[2]*fabsf(R[1][0]) +
	        box2.m_ext[1]*fabsf(R[0][2]) + box2.m_ext[2]*fabsf(R[0][1]) )
		return false;

	// L = A0 x b1
	if( fabsf(T[2]*R[1][1] - T[1]*R[2][1]) >
	        box1.m_ext[1]*fabsf(R[2][1]) + box1.m_ext[2]*fabsf(R[1][1]) +
	        box2.m_ext[0]*fabsf(R[0][2]) + box2.m_ext[2]*fabsf(R[0][0]) )
		return false;

	// L = A0 x B2
	if( fabsf(T[2]*R[1][2] - T[1]*R[2][2]) >
	        box1.m_ext[1]*fabsf(R[2][2]) + box1.m_ext[2]*fabsf(R[1][2]) +
	        box2.m_ext[0]*fabsf(R[0][1]) + box2.m_ext[1]*fabsf(R[0][0]) )
		return false;

	// L = A1 x b0
	if( fabsf(T[0]*R[2][0] - T[2]*R[0][0]) >
	        box1.m_ext[0]*fabsf(R[2][0]) + box1.m_ext[2]*fabsf(R[0][0]) +
	        box2.m_ext[1]*fabsf(R[1][2]) + box2.m_ext[2]*fabsf(R[1][1]) )
		return false;

	// L = A1 x b1
	if( fabsf(T[0]*R[2][1] - T[2]*R[0][1]) >
	        box1.m_ext[0]*fabsf(R[2][1]) + box1.m_ext[2]*fabsf(R[0][1]) +
	        box2.m_ext[0]*fabsf(R[1][2]) + box2.m_ext[2]*fabsf(R[1][0]) )
		return false;

	// L = A1 x B2
	if( fabsf(T[0]*R[2][2] - T[2]*R[0][2]) >
	        box1.m_ext[0]*fabsf(R[2][2]) + box1.m_ext[2]*fabsf(R[0][2]) +
	        box2.m_ext[0]*fabsf(R[1][1]) + box2.m_ext[1]*fabsf(R[1][0]) )
		return false;

	// L = A2 x b0
	if( fabsf(T[1]*R[0][0] - T[0]*R[1][0]) >
	        box1.m_ext[0]*fabsf(R[1][0]) + box1.m_ext[1]*fabsf(R[0][0]) +
	        box2.m_ext[1]*fabsf(R[2][2]) + box2.m_ext[2]*fabsf(R[2][1]) )
		return false;

	// L = A2 x b1
	if( fabsf(T[1]*R[0][1] - T[0]*R[1][1]) >
	        box1.m_ext[0]*fabsf(R[1][1]) + box1.m_ext[1]*fabsf(R[0][1]) +
	        box2.m_ext[0] *fabsf(R[2][2]) + box2.m_ext[2]*fabsf(R[2][0]) )
		return false;

	// L = A2 x B2
	if( fabsf(T[1]*R[0][2] - T[0]*R[1][2]) >
	        box1.m_ext[0]*fabsf(R[1][2]) + box1.m_ext[1]*fabsf(R[0][2]) +
	        box2.m_ext[0]*fabsf(R[2][1]) + box2.m_ext[1]*fabsf(R[2][0]) )
		return false;

	box1.m_ext[0] -= f; box1.m_ext[1] -= f; box1.m_ext[2] -= f;
	box2.m_ext[0] -= f; box2.m_ext[1] -= f; box2.m_ext[2] -= f;
	// no separating axis found, the two boxes overlap
	return true;
}


//-----------------------------------------------------------------------------
// Sphere-OBB
//-----------------------------------------------------------------------------

bool IsIntersecting( const SphereBody& sphere, const BBox& box, const Transform& boxMeshTrans )
{
	// sphere position in box1's frame
	Vector3 pos( box.m_rot * (boxMeshTrans.ToBodySpace(sphere.Position()) - box.m_trans) );

	float s, d = 0;

	// find the square of the distance from the sphere to the box
	if( pos[0] < -box.m_ext[0] )
	{
		s = pos[0] + box.m_ext[0];
		d += s*s; 
	}
	else if( pos[0] > box.m_ext[0] )
	{ 
		s = pos[0] - box.m_ext[0];
		d += s*s; 
	}

	if( pos[1] < -box.m_ext[1] )
	{
		s = pos[1] + box.m_ext[1];
		d += s*s; 
	}
	else if( pos[1] > box.m_ext[1] )
	{ 
		s = pos[1] - box.m_ext[1];
		d += s*s; 
	}

	if( pos[2] < -box.m_ext[2] )
	{
		s = pos[2] + box.m_ext[2];
		d += s*s; 
	}
	else if( pos[2] > box.m_ext[2] )
	{ 
		s = pos[2] - box.m_ext[2];
		d += s*s; 
	}

	return d <= sphere.Radius2();// + COLLISION_EPSILON;
}


//-----------------------------------------------------------------------------
// Point-Triangle
//-----------------------------------------------------------------------------

bool PointInTri( const Tri& tri, const Vector3& point )
{
	Vector3 edges[3] = { tri.p[1] - tri.p[0], tri.p[2] - tri.p[1], tri.p[0] - tri.p[2] };
	Vector3 normal = edges[0].Cross( edges[1] );

	// In the plane of the triangle
	Vector3 cp = edges[0].Cross( point - tri.p[0] );
	normal.Normalize();
	cp.Normalize();
	if( !IsEqual(normal,cp) )
		return false;

	// On the correct side of each edge
	cp = edges[1].Cross( point - tri.p[1] );
	if( normal.Dot(cp) < 0.0 )
		return false;

	cp = edges[2].Cross( point - tri.p[2] );
	if( normal.Dot(cp) < 0.0 )
		return false;

	return true;
}

// This one tests the projection of the point on the triangle's plane
bool PointInTri2D( const Tri& tri, const Vector3& point )
{
	Vector3 edge1 = tri.p[1] - tri.p[0];
	Vector3 edge2 = tri.p[2] - tri.p[0];
	Vector3 normal = edge1.Cross(edge2);

	Vector3 P = normal.Cross( edge2 );
	Vector3 T = point - tri.p[0];
	Vector3 Q = T.Cross( edge1 );

	float det = edge1.Dot( P );

	// calculate U parameter and test bounds
	float u = T.Dot( P );
	if( u < 0.0f || u > det )
		return false;

	// calculate V parameter and test bounds
	float v = normal.Dot( Q );
	if( v < 0.0f || (u + v) > det )
		return false;

	return true;
}


//-----------------------------------------------------------------------------
// Point-Sphere
//-----------------------------------------------------------------------------

bool PointInSphere( const SphereBody& sphere, const Vector3& point )
{
	Vector3 localPoint = point - sphere.Position();
	return localPoint.MagnitudeSquared() <= sphere.Radius2();
}

bool PointOnSphereSurface( const SphereBody& sphere, const Vector3& point )
{
	Vector3 localPoint = point - sphere.Position();
	return IsEqual( localPoint.MagnitudeSquared(), sphere.Radius2() );
}


//-----------------------------------------------------------------------------
// Distances
// All this stuff is from Magic Software
//-----------------------------------------------------------------------------

struct BaryTri
{
	BaryTri( const Tri& t ) : origin(t.p[0]), edge0(t.p[1]-t.p[0]), edge1(t.p[2]-t.p[0]) {}
	Vector3 origin;
	Vector3 edge0;
	Vector3 edge1;
};

struct Segment
{
	Vector3 origin;
	Vector3 dir;
};

static float DistanceSquared( const Vector3& point, const BaryTri& tri, float* pS, float* pT )
{
	Vector3 diff = tri.origin - point;
	float a00 = tri.edge0.MagnitudeSquared();
	float a01 = tri.edge0.Dot(tri.edge1);
	float a11 = tri.edge1.MagnitudeSquared();
	float b0 = diff.Dot(tri.edge0);
	float b1 = diff.Dot(tri.edge1);
	float c = diff.MagnitudeSquared();
	float det = fabsf(a00*a11-a01*a01);
	float s = a01*b1-a11*b0;
	float t = a01*b0-a00*b1;
	float sqrDist;

	if ( s + t <= det )
	{
		if ( s < 0.0f )
		{
			if ( t < 0.0f )  // region 4
			{
				if ( b0 < 0.0f )
				{
					t = 0.0f;
					if ( -b0 >= a00 )
					{
						s = 1.0f;
						sqrDist = a00+2.0f*b0+c;
					}
					else
					{
						s = -b0/a00;
						sqrDist = b0*s+c;
					}
				}
				else
				{
					s = 0.0f;
					if ( b1 >= 0.0f )
					{
						t = 0.0f;
						sqrDist = c;
					}
					else if ( -b1 >= a11 )
					{
						t = 1.0f;
						sqrDist = a11+2.0f*b1+c;
					}
					else
					{
						t = -b1/a11;
						sqrDist = b1*t+c;
					}
				}
			}
			else  // region 3
			{
				s = 0.0f;
				if ( b1 >= 0.0f )
				{
					t = 0.0f;
					sqrDist = c;
				}
				else if ( -b1 >= a11 )
				{
					t = 1.0f;
					sqrDist = a11+2.0f*b1+c;
				}
				else
				{
					t = -b1/a11;
					sqrDist = b1*t+c;
				}
			}
		}
		else if ( t < 0.0f )  // region 5
		{
			t = 0.0f;
			if ( b0 >= 0.0f )
			{
				s = 0.0f;
				sqrDist = c;
			}
			else if ( -b0 >= a00 )
			{
				s = 1.0f;
				sqrDist = a00+2.0f*b0+c;
			}
			else
			{
				s = -b0/a00;
				sqrDist = b0*s+c;
			}
		}
		else  // region 0
		{
			// minimum at interior point
			float fInvDet = 1.0f/det;
			s *= fInvDet;
			t *= fInvDet;
			sqrDist = s*(a00*s+a01*t+2.0f*b0) +
				t*(a01*s+a11*t+2.0f*b1)+c;
		}
	}
	else
	{
		float fTmp0, fTmp1, fNumer, fDenom;

		if ( s < 0.0f )  // region 2
		{
			fTmp0 = a01 + b0;
			fTmp1 = a11 + b1;
			if ( fTmp1 > fTmp0 )
			{
				fNumer = fTmp1 - fTmp0;
				fDenom = a00-2.0f*a01+a11;
				if ( fNumer >= fDenom )
				{
					s = 1.0f;
					t = 0.0f;
					sqrDist = a00+2.0f*b0+c;
				}
				else
				{
					s = fNumer/fDenom;
					t = 1.0f - s;
					sqrDist = s*(a00*s+a01*t+2.0f*b0) +
						t*(a01*s+a11*t+2.0f*b1)+c;
				}
			}
			else
			{
				s = 0.0f;
				if ( fTmp1 <= 0.0f )
				{
					t = 1.0f;
					sqrDist = a11+2.0f*b1+c;
				}
				else if ( b1 >= 0.0f )
				{
					t = 0.0f;
					sqrDist = c;
				}
				else
				{
					t = -b1/a11;
					sqrDist = b1*t+c;
				}
			}
		}
		else if ( t < 0.0f )  // region 6
		{
			fTmp0 = a01 + b1;
			fTmp1 = a00 + b0;
			if ( fTmp1 > fTmp0 )
			{
				fNumer = fTmp1 - fTmp0;
				fDenom = a00-2.0f*a01+a11;
				if ( fNumer >= fDenom )
				{
					t = 1.0f;
					s = 0.0f;
					sqrDist = a11+2.0f*b1+c;
				}
				else
				{
					t = fNumer/fDenom;
					s = 1.0f - t;
					sqrDist = s*(a00*s+a01*t+2.0f*b0) +
						t*(a01*s+a11*t+2.0f*b1)+c;
				}
			}
			else
			{
				t = 0.0f;
				if ( fTmp1 <= 0.0f )
				{
					s = 1.0f;
					sqrDist = a00+2.0f*b0+c;
				}
				else if ( b0 >= 0.0f )
				{
					s = 0.0f;
					sqrDist = c;
				}
				else
				{
					s = -b0/a00;
					sqrDist = b0*s+c;
				}
			}
		}
		else  // region 1
		{
			fNumer = a11 + b1 - a01 - b0;
			if ( fNumer <= 0.0f )
			{
				s = 0.0f;
				t = 1.0f;
				sqrDist = a11+2.0f*b1+c;
			}
			else
			{
				fDenom = a00-2.0f*a01+a11;
				if ( fNumer >= fDenom )
				{
					s = 1.0f;
					t = 0.0f;
					sqrDist = a00+2.0f*b0+c;
				}
				else
				{
					s = fNumer/fDenom;
					t = 1.0f - s;
					sqrDist = s*(a00*s+a01*t+2.0f*b0) +
						t*(a01*s+a11*t+2.0f*b1)+c;
				}
			}
		}
	}

	if ( pS )
		*pS = s;

	if ( pT )
		*pT = t;

	return fabsf(sqrDist);
}

static float DistanceSquared( const Segment& seg0, const Segment& seg1, float* pS0=0, float* pS1=0 )
{
	Vector3 diff = seg0.origin - seg1.origin;
	float a00 = seg0.dir.MagnitudeSquared();
	float a01 = -seg0.dir.Dot(seg1.dir);
	float a11 = seg1.dir.MagnitudeSquared();
	float b0 = diff.Dot(seg0.dir);
	float c = diff.MagnitudeSquared();
	float det = fabsf(a00*a11-a01*a01);
	float b1, s, t, sqrDist, fTmp;

	if ( det >= FLOAT_TOLERANCE )
	{
		// line segments are not parallel
		b1 = -diff.Dot(seg1.dir);
		s = a01*b1-a11*b0;
		t = a01*b0-a00*b1;

		if ( s >= 0.0f )
		{
			if ( s <= det )
			{
				if ( t >= 0.0f )
				{
					if ( t <= det )  // region 0 (interior)
					{
						// minimum at two interior points of 3D lines
						float fInvDet = 1.0f/det;
						s *= fInvDet;
						t *= fInvDet;
						sqrDist = s*(a00*s+a01*t+2.0f*b0) +
							t*(a01*s+a11*t+2.0f*b1)+c;
					}
					else  // region 3 (side)
					{
						t = 1.0f;
						fTmp = a01+b0;
						if ( fTmp >= 0.0f )
						{
							s = 0.0f;
							sqrDist = a11+2.0f*b1+c;
						}
						else if ( -fTmp >= a00 )
						{
							s = 1.0f;
							sqrDist = a00+a11+c+2.0f*(b1+fTmp);
						}
						else
						{
							s = -fTmp/a00;
							sqrDist = fTmp*s+a11+2.0f*b1+c;
						}
					}
				}
				else  // region 7 (side)
				{
					t = 0.0f;
					if ( b0 >= 0.0f )
					{
						s = 0.0f;
						sqrDist = c;
					}
					else if ( -b0 >= a00 )
					{
						s = 1.0f;
						sqrDist = a00+2.0f*b0+c;
					}
					else
					{
						s = -b0/a00;
						sqrDist = b0*s+c;
					}
				}
			}
			else
			{
				if ( t >= 0.0 )
				{
					if ( t <= det )  // region 1 (side)
					{
						s = 1.0f;
						fTmp = a01+b1;
						if ( fTmp >= 0.0f )
						{
							t = 0.0f;
							sqrDist = a00+2.0f*b0+c;
						}
						else if ( -fTmp >= a11 )
						{
							t = 1.0f;
							sqrDist = a00+a11+c+2.0f*(b0+fTmp);
						}
						else
						{
							t = -fTmp/a11;
							sqrDist = fTmp*t+a00+2.0f*b0+c;
						}
					}
					else  // region 2 (corner)
					{
						fTmp = a01+b0;
						if ( -fTmp <= a00 )
						{
							t = 1.0f;
							if ( fTmp >= 0.0f )
							{
								s = 0.0f;
								sqrDist = a11+2.0f*b1+c;
							}
							else
							{
								s = -fTmp/a00;
								sqrDist = fTmp*s+a11+2.0f*b1+c;
							}
						}
						else
						{
							s = 1.0f;
							fTmp = a01+b1;
							if ( fTmp >= 0.0f )
							{
								t = 0.0f;
								sqrDist = a00+2.0f*b0+c;
							}
							else if ( -fTmp >= a11 )
							{
								t = 1.0f;
								sqrDist = a00+a11+c+2.0f*(b0+fTmp);
							}
							else
							{
								t = -fTmp/a11;
								sqrDist = fTmp*t+a00+2.0f*b0+c;
							}
						}
					}
				}
				else  // region 8 (corner)
				{
					if ( -b0 < a00 )
					{
						t = 0.0f;
						if ( b0 >= 0.0f )
						{
							s = 0.0f;
							sqrDist = c;
						}
						else
						{
							s = -b0/a00;
							sqrDist = b0*s+c;
						}
					}
					else
					{
						s = 1.0f;
						fTmp = a01+b1;
						if ( fTmp >= 0.0f )
						{
							t = 0.0f;
							sqrDist = a00+2.0f*b0+c;
						}
						else if ( -fTmp >= a11 )
						{
							t = 1.0f;
							sqrDist = a00+a11+c+2.0f*(b0+fTmp);
						}
						else
						{
							t = -fTmp/a11;
							sqrDist = fTmp*t+a00+2.0f*b0+c;
						}
					}
				}
			}
		}
		else 
		{
			if ( t >= 0.0f )
			{
				if ( t <= det )  // region 5 (side)
				{
					s = 0.0f;
					if ( b1 >= 0.0f )
					{
						t = 0.0f;
						sqrDist = c;
					}
					else if ( -b1 >= a11 )
					{
						t = 1.0f;
						sqrDist = a11+2.0f*b1+c;
					}
					else
					{
						t = -b1/a11;
						sqrDist = b1*t+c;
					}
				}
				else  // region 4 (corner)
				{
					fTmp = a01+b0;
					if ( fTmp < 0.0f )
					{
						t = 1.0f;
						if ( -fTmp >= a00 )
						{
							s = 1.0f;
							sqrDist = a00+a11+c+2.0f*(b1+fTmp);
						}
						else
						{
							s = -fTmp/a00;
							sqrDist = fTmp*s+a11+2.0f*b1+c;
						}
					}
					else
					{
						s = 0.0f;
						if ( b1 >= 0.0f )
						{
							t = 0.0f;
							sqrDist = c;
						}
						else if ( -b1 >= a11 )
						{
							t = 1.0f;
							sqrDist = a11+2.0f*b1+c;
						}
						else
						{
							t = -b1/a11;
							sqrDist = b1*t+c;
						}
					}
				}
			}
			else   // region 6 (corner)
			{
				if ( b0 < 0.0f )
				{
					t = 0.0f;
					if ( -b0 >= a00 )
					{
						s = 1.0f;
						sqrDist = a00+2.0f*b0+c;
					}
					else
					{
						s = -b0/a00;
						sqrDist = b0*s+c;
					}
				}
				else
				{
					s = 0.0f;
					if ( b1 >= 0.0f )
					{
						t = 0.0f;
						sqrDist = c;
					}
					else if ( -b1 >= a11 )
					{
						t = 1.0f;
						sqrDist = a11+2.0f*b1+c;
					}
					else
					{
						t = -b1/a11;
						sqrDist = b1*t+c;
					}
				}
			}
		}
	}
	else
	{
		// line segments are parallel
		if ( a01 > 0.0f )
		{
			// direction vectors form an obtuse angle
			if ( b0 >= 0.0f )
			{
				s = 0.0f;
				t = 0.0f;
				sqrDist = c;
			}
			else if ( -b0 <= a00 )
			{
				s = -b0/a00;
				t = 0.0f;
				sqrDist = b0*s+c;
			}
			else
			{
				b1 = -diff.Dot(seg1.dir);
				s = 1.0f;
				fTmp = a00+b0;
				if ( -fTmp >= a01 )
				{
					t = 1.0f;
					sqrDist = a00+a11+c+2.0f*(a01+b0+b1);
				}
				else
				{
					t = -fTmp/a01;
					sqrDist = a00+2.0f*b0+c+t*(a11*t+2.0f*(a01+b1));
				}
			}
		}
		else
		{
			// direction vectors form an acute angle
			if ( -b0 >= a00 )
			{
				s = 1.0f;
				t = 0.0f;
				sqrDist = a00+2.0f*b0+c;
			}
			else if ( b0 <= 0.0f )
			{
				s = -b0/a00;
				t = 0.0f;
				sqrDist = b0*s+c;
			}
			else
			{
				b1 = -diff.Dot(seg1.dir);
				s = 0.0f;
				if ( b0 >= -a01 )
				{
					t = 1.0f;
					sqrDist = a11+2.0f*b1+c;
				}
				else
				{
					t = -b0/a01;
					sqrDist = c+t*(2.0f*b1+a11*t);
				}
			}
		}
	}

	if ( pS0 )
		*pS0 = s;

	if ( pS1 )
		*pS1 = t;

	return fabsf(sqrDist);
}

static float DistanceSquared( const Segment& seg, const BaryTri& tri, float* pSegS=0, float* pTriU=0, float* pTriV=0 )
{
	Vector3 diff = tri.origin - seg.origin;
	float a00 = seg.dir.MagnitudeSquared();
	float a01 = -seg.dir.Dot(tri.edge0);
	float a02 = -seg.dir.Dot(tri.edge1);
	float a11 = tri.edge0.MagnitudeSquared();
	float a12 = tri.edge0.Dot(tri.edge1);
	float a22 = tri.edge1.Dot(tri.edge1);
	float b0  = -diff.Dot(seg.dir);
	float b1  = diff.Dot(tri.edge0);
	float b2  = diff.Dot(tri.edge1);

	Segment triSeg;
	Vector3 pt;
	float sqrDist, sqrDist0, r, s, t, r0, s0, t0;

	// Set up for a relative error test on the angle between ray direction
	// and triangle normal to determine parallel/nonparallel status.
	Vector3 n = tri.edge0.Cross(tri.edge1);
	float nSqrLen = n.MagnitudeSquared();
	float dot = seg.dir.Dot(n);
	bool bNotParallel = (dot*dot >= FLOAT_TOLERANCE*a00*nSqrLen);

	if ( bNotParallel )
	{
		float fCof00 = a11*a22-a12*a12;
		float fCof01 = a02*a12-a01*a22;
		float fCof02 = a01*a12-a02*a11;
		float fCof11 = a00*a22-a02*a02;
		float fCof12 = a02*a01-a00*a12;
		float fCof22 = a00*a11-a01*a01;
		float fInvDet = 1.0f/(a00*fCof00+a01*fCof01+a02*fCof02);
		float fRhs0 = -b0*fInvDet;
		float fRhs1 = -b1*fInvDet;
		float fRhs2 = -b2*fInvDet;

		r = fCof00*fRhs0+fCof01*fRhs1+fCof02*fRhs2;
		s = fCof01*fRhs0+fCof11*fRhs1+fCof12*fRhs2;
		t = fCof02*fRhs0+fCof12*fRhs1+fCof22*fRhs2;

		if ( r < 0.0f )
		{
			if ( s+t <= 1.0f )
			{
				if ( s < 0.0f )
				{
					if ( t < 0.0f )  // region 4m
					{
						// min on face s=0 or t=0 or r=0
						triSeg.origin = tri.origin;
						triSeg.dir = tri.edge1;
						sqrDist = DistanceSquared(seg,triSeg,&r,&t);
						s = 0.0f;
						triSeg.origin = tri.origin;
						triSeg.dir = tri.edge0;
						sqrDist0 = DistanceSquared(seg,triSeg,&r0,&s0);
						t0 = 0.0f;
						if ( sqrDist0 < sqrDist )
						{
							sqrDist = sqrDist0;
							r = r0;
							s = s0;
							t = t0;
						}
						sqrDist0 = DistanceSquared(seg.origin,tri,&s0,
							&t0);
						r0 = 0.0f;
						if ( sqrDist0 < sqrDist )
						{
							sqrDist = sqrDist0;
							r = r0;
							s = s0;
							t = t0;
						}
					}
					else  // region 3m
					{
						// min on face s=0 or r=0
						triSeg.origin = tri.origin;
						triSeg.dir = tri.edge1;
						sqrDist = DistanceSquared(seg,triSeg,&r,&t);
						s = 0.0f;
						sqrDist0 = DistanceSquared(seg.origin,tri,&s0,
							&t0);
						r0 = 0.0f;
						if ( sqrDist0 < sqrDist )
						{
							sqrDist = sqrDist0;
							r = r0;
							s = s0;
							t = t0;
						}
					}
				}
				else if ( t < 0.0f )  // region 5m
				{
					// min on face t=0 or r=0
					triSeg.origin = tri.origin;
					triSeg.dir = tri.edge0;
					sqrDist = DistanceSquared(seg,triSeg,&r,&s);
					t = 0.0f;
					sqrDist0 = DistanceSquared(seg.origin,tri,&s0,&t0);
					r0 = 0.0f;
					if ( sqrDist0 < sqrDist )
					{
						sqrDist = sqrDist0;
						r = r0;
						s = s0;
						t = t0;
					}
				}
				else  // region 0m
				{
					// min on face r=0
					sqrDist = DistanceSquared(seg.origin,tri,&s,&t);
					r = 0.0f;
				}
			}
			else
			{
				if ( s < 0.0f )  // region 2m
				{
					// min on face s=0 or s+t=1 or r=0
					triSeg.origin = tri.origin;
					triSeg.dir = tri.edge1;
					sqrDist = DistanceSquared(seg,triSeg,&r,&t);
					s = 0.0f;
					triSeg.origin = tri.origin+tri.edge0;
					triSeg.dir = tri.edge1-tri.edge0;
					sqrDist0 = DistanceSquared(seg,triSeg,&r0,&t0);
					s0 = 1.0f-t0;
					if ( sqrDist0 < sqrDist )
					{
						sqrDist = sqrDist0;
						r = r0;
						s = s0;
						t = t0;
					}
					sqrDist0 = DistanceSquared(seg.origin,tri,&s0,&t0);
					r0 = 0.0f;
					if ( sqrDist0 < sqrDist )
					{
						sqrDist = sqrDist0;
						r = r0;
						s = s0;
						t = t0;
					}
				}
				else if ( t < 0.0f )  // region 6m
				{
					// min on face t=0 or s+t=1 or r=0
					triSeg.origin = tri.origin;
					triSeg.dir = tri.edge0;
					sqrDist = DistanceSquared(seg,triSeg,&r,&s);
					t = 0.0f;
					triSeg.origin = tri.origin+tri.edge0;
					triSeg.dir = tri.edge1-tri.edge0;
					sqrDist0 = DistanceSquared(seg,triSeg,&r0,&t0);
					s0 = 1.0f-t0;
					if ( sqrDist0 < sqrDist )
					{
						sqrDist = sqrDist0;
						r = r0;
						s = s0;
						t = t0;
					}
					sqrDist0 = DistanceSquared(seg.origin,tri,&s0,&t0);
					r0 = 0.0f;
					if ( sqrDist0 < sqrDist )
					{
						sqrDist = sqrDist0;
						r = r0;
						s = s0;
						t = t0;
					}
				}
				else  // region 1m
				{
					// min on face s+t=1 or r=0
					triSeg.origin = tri.origin+tri.edge0;
					triSeg.dir = tri.edge1-tri.edge0;
					sqrDist = DistanceSquared(seg,triSeg,&r,&t);
					s = 1.0f-t;
					sqrDist0 = DistanceSquared(seg.origin,tri,&s0,&t0);
					r0 = 0.0f;
					if ( sqrDist0 < sqrDist )
					{
						sqrDist = sqrDist0;
						r = r0;
						s = s0;
						t = t0;
					}
				}
			}
		}
		else if ( r <= 1.0f )
		{
			if ( s+t <= 1.0f )
			{
				if ( s < 0.0f )
				{
					if ( t < 0.0f )  // region 4
					{
						// min on face s=0 or t=0
						triSeg.origin = tri.origin;
						triSeg.dir = tri.edge1;
						sqrDist = DistanceSquared(seg,triSeg,&r,&t);
						s = 0.0f;
						triSeg.origin = tri.origin;
						triSeg.dir = tri.edge0;
						sqrDist0 = DistanceSquared(seg,triSeg,&r0,&s0);
						t0 = 0.0f;
						if ( sqrDist0 < sqrDist )
						{
							sqrDist = sqrDist0;
							r = r0;
							s = s0;
							t = t0;
						}
					}
					else  // region 3
					{
						// min on face s=0
						triSeg.origin = tri.origin;
						triSeg.dir = tri.edge1;
						sqrDist = DistanceSquared(seg,triSeg,&r,&t);
						s = 0.0f;
					}
				}
				else if ( t < 0.0f )  // region 5
				{
					// min on face t=0
					triSeg.origin = tri.origin;
					triSeg.dir = tri.edge0;
					sqrDist = DistanceSquared(seg,triSeg,&r,&s);
					t = 0.0f;
				}
				else  // region 0
				{
					// global minimum is interior, done
					sqrDist = 0.0f;
				}
			}
			else
			{
				if ( s < 0.0f )  // region 2
				{
					// min on face s=0 or s+t=1
					triSeg.origin = tri.origin;
					triSeg.dir = tri.edge1;
					sqrDist = DistanceSquared(seg,triSeg,&r,&t);
					s = 0.0f;
					triSeg.origin = tri.origin+tri.edge0;
					triSeg.dir = tri.edge1-tri.edge0;
					sqrDist0 = DistanceSquared(seg,triSeg,&r0,&t0);
					s0 = 1.0f-t0;
					if ( sqrDist0 < sqrDist )
					{
						sqrDist = sqrDist0;
						r = r0;
						s = s0;
						t = t0;
					}
				}
				else if ( t < 0.0f )  // region 6
				{
					// min on face t=0 or s+t=1
					triSeg.origin = tri.origin;
					triSeg.dir = tri.edge0;
					sqrDist = DistanceSquared(seg,triSeg,&r,&s);
					t = 0.0f;
					triSeg.origin = tri.origin+tri.edge0;
					triSeg.dir = tri.edge1-tri.edge0;
					sqrDist0 = DistanceSquared(seg,triSeg,&r0,&t0);
					s0 = 1.0f-t0;
					if ( sqrDist0 < sqrDist )
					{
						sqrDist = sqrDist0;
						r = r0;
						s = s0;
						t = t0;
					}
				}
				else  // region 1
				{
					// min on face s+t=1
					triSeg.origin = tri.origin+tri.edge0;
					triSeg.dir = tri.edge1-tri.edge0;
					sqrDist = DistanceSquared(seg,triSeg,&r,&t);
					s = 1.0f-t;
				}
			}
		}
		else  // r > 1
		{
			if ( s+t <= 1.0f )
			{
				if ( s < 0.0f )
				{
					if ( t < 0.0f )  // region 4p
					{
						// min on face s=0 or t=0 or r=1
						triSeg.origin = tri.origin;
						triSeg.dir = tri.edge1;
						sqrDist = DistanceSquared(seg,triSeg,&r,&t);
						s = 0.0f;
						triSeg.origin = tri.origin;
						triSeg.dir = tri.edge0;
						sqrDist0 = DistanceSquared(seg,triSeg,&r0,&s0);
						t0 = 0.0f;
						if ( sqrDist0 < sqrDist )
						{
							sqrDist = sqrDist0;
							r = r0;
							s = s0;
							t = t0;
						}
						pt = seg.origin+seg.dir;
						sqrDist0 = DistanceSquared(pt,tri,&s0,&t0);
						r0 = 1.0f;
						if ( sqrDist0 < sqrDist )
						{
							sqrDist = sqrDist0;
							r = r0;
							s = s0;
							t = t0;
						}
					}
					else  // region 3p
					{
						// min on face s=0 or r=1
						triSeg.origin = tri.origin;
						triSeg.dir = tri.edge1;
						sqrDist = DistanceSquared(seg,triSeg,&r,&t);
						s = 0.0f;
						pt = seg.origin+seg.dir;
						sqrDist0 = DistanceSquared(pt,tri,&s0,&t0);
						r0 = 1.0f;
						if ( sqrDist0 < sqrDist )
						{
							sqrDist = sqrDist0;
							r = r0;
							s = s0;
							t = t0;
						}
					}
				}
				else if ( t < 0.0f )  // region 5p
				{
					// min on face t=0 or r=1
					triSeg.origin = tri.origin;
					triSeg.dir = tri.edge0;
					sqrDist = DistanceSquared(seg,triSeg,&r,&s);
					t = 0.0f;
					pt = seg.origin+seg.dir;
					sqrDist0 = DistanceSquared(pt,tri,&s0,&t0);
					r0 = 1.0f;
					if ( sqrDist0 < sqrDist )
					{
						sqrDist = sqrDist0;
						r = r0;
						s = s0;
						t = t0;
					}
				}
				else  // region 0p
				{
					// min face on r=1
					pt = seg.origin+seg.dir;
					sqrDist = DistanceSquared(pt,tri,&s,&t);
					r = 1.0f;
				}
			}
			else
			{
				if ( s < 0.0f )  // region 2p
				{
					// min on face s=0 or s+t=1 or r=1
					triSeg.origin = tri.origin;
					triSeg.dir = tri.edge1;
					sqrDist = DistanceSquared(seg,triSeg,&r,&t);
					s = 0.0f;
					triSeg.origin = tri.origin+tri.edge0;
					triSeg.dir = tri.edge1-tri.edge0;
					sqrDist0 = DistanceSquared(seg,triSeg,&r0,&t0);
					s0 = 1.0f-t0;
					if ( sqrDist0 < sqrDist )
					{
						sqrDist = sqrDist0;
						r = r0;
						s = s0;
						t = t0;
					}
					pt = seg.origin+seg.dir;
					sqrDist0 = DistanceSquared(pt,tri,&s0,&t0);
					r0 = 1.0f;
					if ( sqrDist0 < sqrDist )
					{
						sqrDist = sqrDist0;
						r = r0;
						s = s0;
						t = t0;
					}
				}
				else if ( t < 0.0f )  // region 6p
				{
					// min on face t=0 or s+t=1 or r=1
					triSeg.origin = tri.origin;
					triSeg.dir = tri.edge0;
					sqrDist = DistanceSquared(seg,triSeg,&r,&s);
					t = 0.0f;
					triSeg.origin = tri.origin+tri.edge0;
					triSeg.dir = tri.edge1-tri.edge0;
					sqrDist0 = DistanceSquared(seg,triSeg,&r0,&t0);
					s0 = 1.0f-t0;
					if ( sqrDist0 < sqrDist )
					{
						sqrDist = sqrDist0;
						r = r0;
						s = s0;
						t = t0;
					}
					pt = seg.origin+seg.dir;
					sqrDist0 = DistanceSquared(pt,tri,&s0,&t0);
					r0 = 1.0f;
					if ( sqrDist0 < sqrDist )
					{
						sqrDist = sqrDist0;
						r = r0;
						s = s0;
						t = t0;
					}
				}
				else  // region 1p
				{
					// min on face s+t=1 or r=1
					triSeg.origin = tri.origin+tri.edge0;
					triSeg.dir = tri.edge1-tri.edge0;
					sqrDist = DistanceSquared(seg,triSeg,&r,&t);
					s = 1.0f-t;
					pt = seg.origin+seg.dir;
					sqrDist0 = DistanceSquared(pt,tri,&s0,&t0);
					r0 = 1.0f;
					if ( sqrDist0 < sqrDist )
					{
						sqrDist = sqrDist0;
						r = r0;
						s = s0;
						t = t0;
					}
				}
			}
		}
	}
	else
	{
		// segment and triangle are parallel
		triSeg.origin = tri.origin;
		triSeg.dir = tri.edge0;
		sqrDist = DistanceSquared(seg,triSeg,&r,&s);
		t = 0.0f;

		triSeg.dir = tri.edge1;
		sqrDist0 = DistanceSquared(seg,triSeg,&r0,&t0);
		s0 = 0.0f;
		if ( sqrDist0 < sqrDist )
		{
			sqrDist = sqrDist0;
			r = r0;
			s = s0;
			t = t0;
		}

		triSeg.origin = tri.origin + tri.edge0;
		triSeg.dir = tri.edge1 - tri.edge0;
		sqrDist0 = DistanceSquared(seg,triSeg,&r0,&t0);
		s0 = 1.0f-t0;
		if ( sqrDist0 < sqrDist )
		{
			sqrDist = sqrDist0;
			r = r0;
			s = s0;
			t = t0;
		}

		sqrDist0 = DistanceSquared(seg.origin,tri,&s0,&t0);
		r0 = 0.0f;
		if ( sqrDist0 < sqrDist )
		{
			sqrDist = sqrDist0;
			r = r0;
			s = s0;
			t = t0;
		}

		pt = seg.origin+seg.dir;
		sqrDist0 = DistanceSquared(pt,tri,&s0,&t0);
		r0 = 1.0f;
		if ( sqrDist0 < sqrDist )
		{
			sqrDist = sqrDist0;
			r = r0;
			s = s0;
			t = t0;
		}
	}

	if ( pSegS )
		*pSegS = r;

	if ( pTriU )
		*pTriU = s;

	if ( pTriV )
		*pTriV = t;

	return sqrDist;
}

float DistanceSquared( const Tri& tri, const Vector3& point, Vector3* pProjected )
{
	float u, v, ret;
	BaryTri baryTri(tri);
	ret = DistanceSquared( point, baryTri, &u, &v );

	if( pProjected )
	{
		*pProjected = baryTri.origin + u*baryTri.edge0 + v*baryTri.edge1;
	}

	return ret;
}

float DistanceSquared( const Tri& tri0_, const Tri& tri1_, Vector3* pPt1, Vector3* pPt2 )
{
	float s, t, u, v, s0, t0, u0, v0, sqrDist, sqrDist0;
	Segment seg;
	BaryTri tri0(tri0_), tri1(tri1_);

	// compare edges of tri0 against all of tri1
	seg.origin = tri0.origin;
	seg.dir = tri0.edge0;
	sqrDist = DistanceSquared(seg,tri1,&s,&u,&v);
	t = 0.0f;

	seg.dir = tri0.edge1;
	sqrDist0 = DistanceSquared(seg,tri1,&t0,&u0,&v0);
	s0 = 0.0f;
	if ( sqrDist0 < sqrDist )
	{
		sqrDist = sqrDist0;
		s = s0;
		t = t0;
		u = u0;
		v = v0;
	}

	seg.origin = seg.origin + tri0.edge0;
	seg.dir = seg.dir - tri0.edge0;
	sqrDist0 = DistanceSquared(seg,tri1,&t0,&u0,&v0);
	s0 = 1.0f-t0;
	if ( sqrDist0 < sqrDist )
	{
		sqrDist = sqrDist0;
		s = s0;
		t = t0;
		u = u0;
		v = v0;
	}

	// compare edges of tri1 against all of tri0
	seg.origin = tri1.origin;
	seg.dir = tri1.edge0;
	sqrDist0 = DistanceSquared(seg,tri0,&u0,&s0,&t0);
	v0 = 0.0f;
	if ( sqrDist0 < sqrDist )
	{
		sqrDist = sqrDist0;
		s = s0;
		t = t0;
		u = u0;
		v = v0;
	}

	seg.dir = tri1.edge1;
	sqrDist0 = DistanceSquared(seg,tri0,&v0,&s0,&t0);
	u0 = 0.0f;
	if ( sqrDist0 < sqrDist )
	{
		sqrDist = sqrDist0;
		s = s0;
		t = t0;
		u = u0;
		v = v0;
	}

	seg.origin = seg.origin + tri1.edge0;
	seg.dir = seg.dir - tri1.edge0;
	sqrDist0 = DistanceSquared(seg,tri0,&v0,&s0,&t0);
	u0 = 1.0f-v0;
	if ( sqrDist0 < sqrDist )
	{
		sqrDist = sqrDist0;
		s = s0;
		t = t0;
		u = u0;
		v = v0;
	}

	if( pPt1 && pPt2 )
		*pPt1 = tri0.origin + s*tri0.edge0 + t*tri0.edge1;

	if( pPt2 )
		*pPt2 = tri1.origin + u*tri1.edge0 + v*tri1.edge1;

	return fabsf(sqrDist);
}

} // namespace physics
