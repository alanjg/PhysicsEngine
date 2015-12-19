#ifndef SPHEREBODY_H
#define SPHEREBODY_H
#include "RigidBody.h"
#include <gl/glut.h>
#include <iostream>

namespace physics
{

    class SphereBody: public RigidBody
	{
	public:
        SphereBody(const Vector3& position,float radius,bool mobile);
		virtual ~SphereBody();
        virtual SphereBody* Clone();

        float Radius() const { return radius_; }
        float Radius2() const { return radius2_; }
        virtual void Render()
        {
            glPushMatrix();
			Vector3 pos = Position();
            glTranslatef(pos[0],pos[1],pos[2]);
            glutSolidSphere(radius_,100,100);
            glPopMatrix();
        }
        virtual void CopyTo(RigidBody* target)
        {   
            SphereBody* sphereTarget = dynamic_cast<SphereBody*>(target);
            RigidBody::CopyTo(target);
            sphereTarget->radius_ = radius_;
        }
		
		virtual void Print(std::ostream& out) const
		{
			out<<"<SphereBody> pos:"<<Position()<<" vel:"<<velocity_;
		}
    private:
        float radius_;
		float radius2_;
	};

} // namespace physics

#endif