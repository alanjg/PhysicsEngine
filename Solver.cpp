#include "Solver.h"
#include <iostream>
#include <vector>
using namespace std;

namespace physics
{

void Solver::Update(float time)
{
    for(unsigned int i=0;i<simulation_->objects_.size(); i++)
    {
        simulation_->objects_[i]->CopyTo(simulation_->nextObjects_[i]);
    }
	for(vector<RigidBody*>::iterator it = simulation_->nextObjects_.begin(); it!= simulation_->nextObjects_.end(); it++)
	{
		RigidBody& rigidBody = *(*it);
		Vector3 acceleration(0,-3.0f,0);
		
		Vector3 angularAcceleration(0,0,0);

		if(!(*it)->Mobile())
		{
			(*it)->velocity_ = Vector3(0,0,0);
			(*it)->angularVelocity_ = Vector3(0,0,0);
			continue;
		}

		Vector3 deltaV = acceleration * time;
		if(rigidBody.resting_) std::cout <<"deltaV pre"<<deltaV<<std::endl;
		deltaV = rigidBody.ApplyResting(deltaV);
		if(rigidBody.resting_) std::cout <<"deltaV post"<<deltaV<<std::endl;

		Vector3 deltaW = rigidBody.inverseInertia_ * (angularAcceleration - rigidBody.angularVelocity_.Cross(rigidBody.inertia_ * rigidBody.angularVelocity_)) * time;
		if(rigidBody.resting_)
			deltaW.Set(0,0,0);

		//calc velocities
        rigidBody.velocity_ += deltaV;
		rigidBody.angularVelocity_ += deltaW;

		rigidBody.transform_.AddTranslation( rigidBody.velocity_ * time );
		Vector3 angular = rigidBody.angularVelocity_ * time;
		float angle = angular.Magnitude();
		if(angle != 0)
		{
			angular /= angle;
			rigidBody.transform_.AddRotation( angular, angle );
		}
	}
}

void Solver::LockInUpdate()
{
    for(unsigned int i=0;i<simulation_->objects_.size(); i++)
    {
        swap(simulation_->nextObjects_[i],simulation_->objects_[i]);
    }
}

} // namespace physics