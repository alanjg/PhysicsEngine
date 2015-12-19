#include "Collision.h"
#include "RigidBody.h"
#include "PolyMeshBody.h"
#include "SphereBody.h"
#include "Constants.h"
#include "Intersection.h"
#include <iostream>
#include <vector>
#include <set>
#include <utility>
using namespace std;

namespace physics
{

void ResolveCollision(RigidBody* p1,RigidBody* p2,const Collision& collision)
{
	Vector3 localPoint1 = collision.collisionPoint - p1->Position();
	Vector3 localPoint2 = collision.collisionPoint - p2->Position();

	Vector3 localCrossNormal1 = localPoint1.Cross(collision.collisionNormal);
	Vector3 localCrossNormal2 = localPoint2.Cross(collision.collisionNormal);

	float relVel = collision.relativeVelocity.Dot(collision.collisionNormal);
	Vector3 den1 = (p1->InverseInertia() * localCrossNormal1).Cross(localPoint1);
	Vector3 den2 = (p2->InverseInertia() * localCrossNormal2).Cross(localPoint2);
	
	float denComb =  collision.collisionNormal.Dot(den1 + den2);

	float den = denComb + p1->InverseMass() + p2->InverseMass();

	relVel /= den;
	Vector3 impulseVector1 = -(1 + p1->Restitution()) * relVel * collision.collisionNormal;
    Vector3 impulseVector2 = (1 + p2->Restitution()) * relVel * collision.collisionNormal;
	//Vector3 impulseVector1 = impulse * collision.collisionNormal;
	//Vector3 impulseVector2 = -impulseVector1;

	if(!p1->BeginRestingContact(p2,collision.collisionNormal) && !p2->BeginRestingContact(p1,-collision.collisionNormal))
	{
		p1->ApplyLinearImpulse(impulseVector1);
		p2->ApplyLinearImpulse(impulseVector2);

		p1->ApplyAngularImpulse(localPoint1.Cross(impulseVector1));
		p2->ApplyAngularImpulse(localPoint2.Cross(impulseVector2));

		p1->EndRestingContact(p2);
		p2->EndRestingContact(p1);
	}


	
	if(OUTPUT_DEBUG)
	{
		cerr<<"************************************"<< endl;
		cerr<<"Resolving collision between "<<endl<<*p1<<endl<<*p2<<endl;
		cerr<<"RelativeVelocity:"<<collision.relativeVelocity<<endl;
		cerr<<"CollisionNormal:"<<collision.collisionNormal<<endl;
		
		cerr<<"collisionPoint:"<<collision.collisionPoint<<endl;
		cerr<<"localPoint1:"<<localPoint1<<endl;
		cerr<<"localPoint2:"<<localPoint2<<endl;

		cerr<<"localCrossNormal1:"<<localCrossNormal1<<endl;
		cerr<<"localCrossNormal2:"<<localCrossNormal2<<endl;

		cerr<<"relVel:"<<relVel<<endl;

		cerr<<"DenComb:"<<denComb<<endl;

		float impulse = -(1 + p1->Restitution()) * relVel /den;
		cerr<<"impulse is "<<impulse<<endl;



		cerr<<"impulse vector1 is "<<impulseVector1<<endl;
		cerr<<"impulse vector2 is "<<impulseVector2<<endl;

		cerr<<"angular impulse vector2 is "<<(localPoint2.Cross(impulseVector2))<<endl;
		cerr<<"angular velocity is "<<p2->AngularVelocity()<<endl;

		cerr<<"final result:"<<endl<<*p1<<endl<<*p2<<endl;
	}
}


CollisionResult CheckForCollision(SphereBody* p1,SphereBody* p2,Collision& collision)
{
    float distanceSquared = (p1->Position() - p2->Position()).MagnitudeSquared();
    float radiiSquared = (p1->Radius() + p2->Radius()) * (p1->Radius() + p2->Radius());

    if(distanceSquared <= radiiSquared + PENETRATION_EPSILON)
    {
        collision.collisionNormal = p1->Position() - p2->Position();
        collision.collisionNormal.Normalize();

        collision.collisionPoint = p1->Position() + collision.collisionNormal * p1->Radius();

        collision.relativeVelocity = p1->RelativeVelocity(collision.collisionPoint) - p2->RelativeVelocity(collision.collisionPoint);
		if(collision.relativeVelocity.Dot(collision.collisionNormal) < 0.0)
			return PENETRATING;
    }

    if(distanceSquared <= radiiSquared + COLLISION_EPSILON)
    {
        collision.collisionNormal = p1->Position() - p2->Position();
        collision.collisionNormal.Normalize();

        collision.collisionPoint = p1->Position() + collision.collisionNormal * p1->Radius();

        collision.relativeVelocity = p1->RelativeVelocity(collision.collisionPoint) - p2->RelativeVelocity(collision.collisionPoint);
        if(collision.relativeVelocity.Dot(collision.collisionNormal) < 0.0)
			return COLLIDING;
    }

	return NOTTOUCHING;
}

static CollisionResult CheckForCollision( SphereBody* p1, PolyMeshBody* p2, const vector<int>& iColBoxes, Collision& collision )
{
	Vector3 colPt, relPos, projected;
	float distSquared, minDistSquared = 10.0f;
	for( unsigned int i = 0; i < iColBoxes.size(); i++ )
	{
		relPos = p2->ToBodySpace( p1->Position() );
		distSquared = DistanceSquared( p2->GetBoxTri(iColBoxes[i]), relPos, &projected );
		if( distSquared <= minDistSquared )
		{
			colPt = p2->BodyToWorldSpace( projected );
			minDistSquared = distSquared;
		}
	}

	if( minDistSquared <= p1->Radius2() + PENETRATION_EPSILON )
	{
		collision.collisionNormal = p1->Position() - colPt;
		collision.collisionNormal.Normalize();
		collision.collisionPoint = p1->Position() - collision.collisionNormal * p1->Radius();
		collision.relativeVelocity = p1->RelativeVelocity(collision.collisionPoint) - p2->RelativeVelocity(collision.collisionPoint);

		if( collision.relativeVelocity.Dot(collision.collisionNormal) < 0.0 )
		{
			return PENETRATING;
		}
	}
	if( minDistSquared <= p1->Radius2() + COLLISION_EPSILON )
	{
		collision.collisionNormal = p1->Position() - colPt;
		collision.collisionNormal.Normalize();
		collision.collisionPoint = p1->Position() - collision.collisionNormal * p1->Radius();
		collision.relativeVelocity = p1->RelativeVelocity(collision.collisionPoint) - p2->RelativeVelocity(collision.collisionPoint);

		if( collision.relativeVelocity.Dot(collision.collisionNormal) < 0.0 )
		{
			return COLLIDING;
		}
	}

	return NOTTOUCHING;
}

static bool CollideRecursive( SphereBody* sphere, PolyMeshBody* mesh, int iCurBox, vector<int>& collisionBoxes )
{
	if( !IsIntersecting(*sphere, mesh->GetBox(iCurBox), mesh->GetTransform()) )
		return false;

	if( mesh->GetBox(iCurBox).IsLeaf() )
	{
		collisionBoxes.push_back( iCurBox );
		return true;
	}

	bool b1 = CollideRecursive( sphere, mesh, mesh->GetBox(iCurBox).m_iLeft, collisionBoxes );
	bool b2 = CollideRecursive( sphere, mesh, mesh->GetBox(iCurBox).m_iRight, collisionBoxes );
	if( !b1 && !b2 )
		return false;

	return true;
}

CollisionResult CheckForCollision(SphereBody* p1,PolyMeshBody* p2,Collision& collision)
{
	vector<int> collisionBoxes;
    if( CollideRecursive(p1, p2, 0, collisionBoxes) )
    {
		return CheckForCollision( p1, p2, collisionBoxes, collision );
    }

	return NOTTOUCHING;
}

inline CollisionResult CheckForCollision(PolyMeshBody* p1,SphereBody* p2,Collision& collision)
{
	CollisionResult retVal = CheckForCollision( p2, p1, collision );
	collision.collisionNormal = -collision.collisionNormal;
	collision.relativeVelocity = -collision.relativeVelocity;
	return retVal;
}

static CollisionResult CheckForCollision( PolyMeshBody* p1, PolyMeshBody* p2, const vector< pair<int,int> >& iColBoxes, Collision& collision )
{
	Tri t1, t2;
	Vector3 curNormal;
	Vector3 proj1, proj2;
	vector<Tri> transformedTris1( iColBoxes.size() );
	vector<Tri> transformedTris2( iColBoxes.size() );

	collision.collisionPoint.Set(0,0,0);
	collision.collisionNormal.Set(0,0,0);
	int numPenetrations = 0;

	p1->bi.clear();
	p2->bi.clear();
	for( unsigned int i = 0; i < iColBoxes.size(); i++ )
	{
		t1 = p1->GetBoxTri( iColBoxes[i].first );
		t1.p[0] = p1->GetTransform().BodyToWorldSpace( t1.p[0] );
		t1.p[1] = p1->GetTransform().BodyToWorldSpace( t1.p[1] );
		t1.p[2] = p1->GetTransform().BodyToWorldSpace( t1.p[2] );
		t2 = p2->GetBoxTri( iColBoxes[i].second );
		t2.p[0] = p2->GetTransform().BodyToWorldSpace( t2.p[0] );
		t2.p[1] = p2->GetTransform().BodyToWorldSpace( t2.p[1] );
		t2.p[2] = p2->GetTransform().BodyToWorldSpace( t2.p[2] );
		if( IsIntersecting(t1, t2) )
		{
			cout<<"here"<<endl;
			p1->bi.push_back( iColBoxes[i].first );
			p2->bi.push_back( iColBoxes[i].second );
			DistanceSquared( t1, t2, &proj1, &proj2 );
			curNormal = ( t2.p[1] - t2.p[0] ).Cross( t2.p[2] - t2.p[0] )
			            - ( t1.p[1] - t1.p[0] ).Cross( t1.p[2] - t1.p[0] );
			curNormal.Normalize();
			collision.collisionNormal += curNormal;
			collision.collisionPoint += ( proj1 + proj2 ) / 2.0f;
			numPenetrations++;
		}
		transformedTris1[i] = t1;
		transformedTris2[i] = t2;
	}

	if( numPenetrations )
	{
		collision.collisionPoint /= (float)numPenetrations;
		collision.collisionNormal.Normalize();
		collision.relativeVelocity = p1->RelativeVelocity(collision.collisionPoint) - p2->RelativeVelocity(collision.collisionPoint);
		if(collision.relativeVelocity.Dot(collision.collisionNormal) < 0.0)
		{
			//cerr<<"penetrating with ok relVel" << endl;
			return PENETRATING;
		}
		//cerr<<"penetrating with bad relVel" << endl;
		return NOTTOUCHING;
	}

	float distSquared, minDistSquared = numeric_limits<float>::max();
	for( unsigned int i = 0; i < iColBoxes.size(); i++ )
	{
		distSquared = DistanceSquared( transformedTris1[i], transformedTris2[i], &proj1, &proj2 );
		if( distSquared <= minDistSquared )
		{
			collision.collisionPoint = 0.5f * (proj1 + proj2);
			collision.collisionNormal = proj1 - proj2;
			minDistSquared = distSquared;
		}
	}

	if( minDistSquared <= COLLISION_EPSILON )
	{
		collision.collisionNormal.Normalize();
		collision.relativeVelocity = p1->RelativeVelocity(collision.collisionPoint) - p2->RelativeVelocity(collision.collisionPoint);
		if(collision.relativeVelocity.Dot(collision.collisionNormal) < 0.0)
		{
			//cerr<<"colliding with ok relVel" << endl;
			return COLLIDING;
		}
		//cerr<<"colliding with bad relVel" << endl;
	}

	return NOTTOUCHING;
}

static bool CollideRecursive( const PolyMeshBody* p1, const PolyMeshBody* p2, int iCurBox1, int iCurBox2, vector< pair<int,int> >& collisionBoxes, float f )
{
	const BBox* pB1 = &p1->GetBox( iCurBox1 );
	const BBox* pB2 = &p2->GetBox( iCurBox2 );
	if( !IsIntersecting(*pB1, *pB2, p1->GetTransform(), p2->GetTransform(), f) )
		return false;

	if( pB1->IsLeaf() && pB2->IsLeaf() )
	{
		collisionBoxes.push_back( make_pair(iCurBox1, iCurBox2) );
		return true;
	}

	if( pB2->IsLeaf() || (!pB1->IsLeaf() && (pB1->m_ext[0] > pB2->m_ext[0])) )
	{
		bool bRet1 = CollideRecursive( p1, p2, pB1->m_iLeft, iCurBox2, collisionBoxes, f );
		bool bRet2 = CollideRecursive( p1, p2, pB1->m_iRight, iCurBox2, collisionBoxes, f );
		if( !bRet1 && !bRet2 )
			return false;
		return true;
	}
	//else
	{
		bool bRet1 = CollideRecursive( p1, p2, iCurBox1, pB2->m_iLeft, collisionBoxes, f );
		bool bRet2 = CollideRecursive( p1, p2, iCurBox1, pB2->m_iRight, collisionBoxes, f );
		if( !bRet1 && !bRet2 )
			return false;
		return true;
	}
}

static CollisionResult CheckForCollision(PolyMeshBody* p1,PolyMeshBody* p2,Collision& collision)
{
	vector< pair<int,int> > collisionBoxes;
	if( CollideRecursive(p1, p2, 0, 0, collisionBoxes, 0) )
	{
		return CheckForCollision( p1, p2, collisionBoxes, collision );
	}

	return NOTTOUCHING;
}

CollisionResult CheckForCollision(RigidBody* a,RigidBody* b,Collision& collision)
{
	SphereBody* sphereBody1 = dynamic_cast<SphereBody*>(a);
    if(sphereBody1)
    {
	    SphereBody* sphereBody2 = dynamic_cast<SphereBody*>(b);
        if(sphereBody2)
        {
            return CheckForCollision(sphereBody1,sphereBody2,collision);
        }
        PolyMeshBody* polyMeshBody = dynamic_cast<PolyMeshBody*>(b);
        if(polyMeshBody)
        {
            return CheckForCollision(sphereBody1,polyMeshBody,collision);
        }
    }
    PolyMeshBody* polyMeshBody1 = dynamic_cast<PolyMeshBody*>(a);
    if(polyMeshBody1)
    {
        SphereBody* sphereBody = dynamic_cast<SphereBody*>(b);
        if(sphereBody)
        {
            return CheckForCollision(polyMeshBody1,sphereBody,collision);
        }
        PolyMeshBody* polyMeshBody2 = dynamic_cast<PolyMeshBody*>(b);
        if(polyMeshBody2)
        {
            return CheckForCollision(polyMeshBody1,polyMeshBody2,collision);
        }
    }
    return NOTTOUCHING;    
}

} // namespace physics