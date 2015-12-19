//-----------------------------------------------------------------------------
// Intersections.h
//
// Intersection tests for objects of various types
//-----------------------------------------------------------------------------

#ifndef INTERSECTION_H
#define INTERSECTION_H

#include "geom.h"
#include "mathdefs.h"

namespace physics
{

	class SphereBody;
	class PolyMeshBody;
	class Transform;

	bool IsIntersecting( const Tri& tri1, const Tri& tri2 );
	bool IsIntersecting( const BBox& box1, const BBox& box2, const Transform& mesh1Trans, const Transform& mesh2Trans, float f );
	bool IsIntersecting( const SphereBody& sphere, const BBox& box, const Transform& boxMeshTrans );

	bool PointInTri( const Tri& tri, const Vector3& point );
	bool PointInTri2D( const Tri& tri, const Vector3& point );

	bool PointInSphere( const SphereBody& sphere, const Vector3& point );
	bool PointOnSphereSurface( const SphereBody& sphere, const Vector3& point );

	float DistanceSquared( const Tri& tri, const Vector3& point, Vector3* pProjected=0 );
	float DistanceSquared( const Tri& tri0, const Tri& tri1, Vector3* pPt1=0, Vector3* pPt2=0 );

} // namespace physics

#endif  // INTERSECTION_H