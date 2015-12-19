#ifndef COLLISION_H
#define COLLISION_H

#include "Vector3.h"

namespace physics
{
    class RigidBody;

    struct Collision
    {
        Vector3 collisionNormal; //towards first
        Vector3 relativeVelocity; //relative to first
        Vector3 collisionPoint; //world space
    };

    struct CollisionInfo
    {
        CollisionInfo(Collision col,RigidBody* p1,RigidBody* p2)
        {
            collision = col;
            a = p1;
            b = p2;
        }
        Collision collision;
        RigidBody* a;
        RigidBody* b;
    };

	enum CollisionResult { NOTTOUCHING, COLLIDING, PENETRATING};

	void ResolveCollision(RigidBody* a,RigidBody* b,const Collision& collision);
	CollisionResult CheckForCollision(RigidBody* a,RigidBody* b,Collision& collision);

} // namespace physics

#endif