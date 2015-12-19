#include "SphereBody.h"

namespace physics
{

SphereBody::SphereBody(const Vector3& position,float radius,bool mobile):
    RigidBody(position,Rotation(0,0,0,1),mobile), radius_(radius)
{
	radius2_ = radius_ * radius_;
}

SphereBody::~SphereBody()
{
    
}

SphereBody* SphereBody::Clone()
{
    SphereBody* p = new SphereBody(*this);
    return p;
}

} // namespace physics