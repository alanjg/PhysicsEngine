#include "Simulation.h"
#include "Collision.h"
#include "Constants.h"
#include <list>
#include <algorithm>
#include <iostream>
#include <cassert>
using namespace std;

namespace physics
{

Simulation::Simulation()
{
	solver_ = new Solver(this);
}

Simulation::~Simulation()
{
	for( size_t i = 0; i < objects_.size(); i++ )
	{
		delete objects_[i];
		delete nextObjects_[i];
	}
}

void Simulation::AddObject(RigidBody* object)
{
	objects_.push_back(object->Clone());
	nextObjects_.push_back(object->Clone());
}

void Simulation::Render()
{
	for (auto ptr = objects_.begin(); ptr != objects_.end(); ptr++)
	{
		(*ptr)->Render();
	}
}

void Simulation::Update(float time,int depth)
{
	if(depth > 100 || time < TIME_EPSILON)
		return;

	solver_->Update(time);

	std::list<CollisionInfo> collisions;
	//std::list<CollisionInfo> penetrations;

	for(unsigned int i=0; i+1 < nextObjects_.size(); i++)
	{
		for(unsigned int j=i+1; j < nextObjects_.size(); j++)
		{
			Collision collision;
			if(nextObjects_[i]->CanCollide(nextObjects_[j]) && nextObjects_[j]->CanCollide(nextObjects_[i]))
			{
				if(nextObjects_[i]->Resting() && nextObjects_[j]->Resting())
					cout<<"both resting"<<endl;

				CollisionResult result = CheckForCollision(nextObjects_[i],nextObjects_[j],collision);
				switch(result)
				{
				case NOTTOUCHING:
					break;
				case COLLIDING:
					collisions.push_back(CollisionInfo(collision,nextObjects_[i],nextObjects_[j]));
					break;
				case PENETRATING:
					//penetrations.push_back(CollisionInfo(collision,nextObjects_[i],nextObjects_[j]));
					collisions.push_back(CollisionInfo(collision,nextObjects_[i],nextObjects_[j]));
					break;
				}
			}
		}
	}
	//if(!penetrations.empty())
	//{
	//	list<CollisionInfo>::iterator it = penetrations.begin();
	//	float bestTime = BinarySearch(0,time,it->a,it->b);
	//	list<CollisionInfo>::iterator bestIt = it;
	//	for(++it; it != penetrations.end(); ++it)
	//	{
	//		float thisTime = BinarySearch(0,time,it->a,it->b);
	//		if(thisTime < bestTime)
	//		{
	//			bestTime = thisTime;
	//			bestIt = it;
	//		}
	//	}

	//	//resolve this collision
	//	solver_->Update(bestTime);

	//	Collision collision;
	//	collision.collisionNormal.Set(0,0,0);
	//	CheckForCollision(bestIt->a,bestIt->b,collision);
	//	ResolveCollision(bestIt->a,bestIt->b,collision);
	//	solver_->LockInUpdate();

	//	Update(time - bestTime,depth+1);
	//	return;
	//}
	for(list<CollisionInfo>::iterator it = collisions.begin(); it != collisions.end(); ++it)
	{
		ResolveCollision(it->a,it->b,it->collision);
	}
	solver_->LockInUpdate();
}
/*
float Simulation::BinarySearch(float begin,float end,RigidBody* a,RigidBody* b)
{
	if(end - begin < TIME_EPSILON)
	{
		return (begin + end) / 2.0f;
	}

	Collision collision;

	//check begin
	//solver_->Update(begin + TIME_EPSILON);
	solver_->Update(begin);
	CollisionResult result = CheckForCollision(a,b,collision);
	switch(result)
	{
	case NOTTOUCHING:
		break;
	case COLLIDING:
		//return begin + TIME_EPSILON;
		return begin;
		break;
	case PENETRATING:
		//push faster one back slightly
		if(fabs(a->Velocity().MagnitudeSquared()) > fabs(b->Velocity().MagnitudeSquared()))
		{
			a->Displace(collision.collisionNormal * -1 * COLLISION_EPSILON);
		}
		else
		{
			b->Displace(collision.collisionNormal * COLLISION_EPSILON);
		}

	assert( true );
		//return begin + TIME_EPSILON;
		return begin;
		break;
	}

	//check end
	//solver_->Update(end - TIME_EPSILON);
	solver_->Update(end);
	result = CheckForCollision(a,b,collision);
	switch(result)
	{
	case NOTTOUCHING:
		//push faster one back slightly
		//-- HUH?! What are we doing here? Also, collision isn't filled out if we're NOTTOUCHING!
		//if(fabs(a->Velocity().MagnitudeSquared()) > fabs(b->Velocity().MagnitudeSquared()))
		//{
		//    a->Displace(collision.collisionNormal * -1 * COLLISION_EPSILON);
		//}
		//else
		//{
		//    b->Displace(collision.collisionNormal * COLLISION_EPSILON);
		//}

	assert( true );
		return end;
		break;
	case COLLIDING:
		//return end - TIME_EPSILON;
		return end;
		break;
	case PENETRATING:
		break;
	}

	solver_->Update((begin + end) / 2.0f);

	result = CheckForCollision(a,b,collision);
	switch(result)
	{
	case NOTTOUCHING:
		return BinarySearch((begin + end) / 2.0f,end,a,b);
		break;
	case COLLIDING:
		return (begin + end) / 2.0f;
		break;
	case PENETRATING:
		return BinarySearch(begin,(begin + end) / 2.0f,a,b);
		break;
	}
	assert( true );
	return (begin + end) / 2.0f;
}
*/
} // namespace physics