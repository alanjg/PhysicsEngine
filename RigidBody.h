#ifndef RIGIDBODY_H
#define RIGIDBODY_H
#include "Vector3.h"
#include "Matrix33.h"
#include "Matrix44.h"
#include "Rotation.h"
#include "Constants.h"
#include <iosfwd>
#include <iostream>
#include <limits>

namespace physics
{
	class Transform
	{
	public:
		Transform() {}
		explicit Transform( const Vector3& trans ) : m_translation(trans), m_rotation(Vector3f(0,0,1),0) {}
		Transform( const Vector3& trans, const Rotation& rot ) : m_translation(trans), m_rotation(rot) {}

		const Vector3&  GetTranslation() const { return m_translation; }
		const Rotation& GetRotation()    const { return m_rotation; }

		void SetTranslation( const Vector3& trans ) { m_translation = trans; }
		void SetRotation   ( const Rotation& rot  ) { m_rotation = rot; }

		void AddTranslation( const Vector3& trans ) { m_translation += trans; }
		void AddRotation   ( const Rotation& rot  ) { m_rotation *= rot; }
		void AddRotation   ( const Vector3& axis, const float angle ) { m_rotation *= Rotation(axis,angle); }

		Vector3 ToBodySpace( const Vector3& worldSpace ) const
		{
			return m_rotation.Inverse().Rotate( worldSpace - m_translation );
		}
		Vector3 BodyToWorldSpace( const Vector3& bodySpace ) const
		{
			return m_rotation.Rotate( bodySpace ) + m_translation;
		}



	private:
		Vector3  m_translation;
		Rotation m_rotation;
	};

	class RigidBody
	{
		friend class Solver;
		static int ids;
		int id;
	public:
		
        RigidBody(const Vector3& position, const Rotation& rot, bool mobile):
            transform_(position,rot),velocity_(0,0,0),inverseMass_(0.1f),restitution_(0.8f),angularVelocity_(0,0,0)
        {
			id = ids++;
			restingBodyId = -1;
			mobile_ = mobile;
			resting_ = !mobile;
			inertia_.MakeIdentity();
			inverseInertia_.MakeIdentity();
			//inertia_ *= 1000;
			//inverseInertia_ *= 0.001f;
			if(!mobile)
			{
				inverseMass_ = 0;
				inertia_ *= std::numeric_limits<float>::quiet_NaN();
				inverseInertia_ *= 0;
			}
        }

		virtual ~RigidBody()=0 {};
        virtual RigidBody* Clone()=0;

		const Vector3& Position() const { return transform_.GetTranslation(); }
		const Rotation& Orientation() const { return transform_.GetRotation(); }
		const Transform& GetTransform() const { return transform_; }
        const Vector3& Velocity() const { return velocity_; }
        const Vector3& AngularVelocity() const { return angularVelocity_; }

        const float InverseMass() const { return inverseMass_; }
		const float Restitution() const { return restitution_; }
		const Matrix33 InverseInertia() const { return inverseInertia_; }

        void ApplyLinearImpulse(const Vector3& impulse)
        {
			if(mobile_)
			{
				Vector3 imp = ApplyResting(impulse);
				velocity_ += imp * inverseMass_;
			}
        }

		void ApplyAngularImpulse(const Vector3& impulse)
        {
			if(mobile_)
			{
				angularVelocity_ += inverseInertia_ * impulse;
			}
        }

        void Displace(const Vector3& displacement)
        {
			transform_.AddTranslation( displacement );
        }
		void Displace(const Vector3& axis, const float angle)
		{
			transform_.AddRotation( axis, angle );
		}

        Vector3 RelativeVelocity(const Vector3& worldSpace) const
        {
			Vector3 objectSpace = worldSpace - Position();
            Vector3 relativeVelocity = velocity_ - objectSpace.Cross(angularVelocity_);
            return relativeVelocity;
        }

		Vector3 ToBodySpace( const Vector3& worldSpace ) const
		{
			return transform_.ToBodySpace( worldSpace );
		}
		Vector3 BodyToWorldSpace( const Vector3& bodySpace ) const
		{
			return transform_.BodyToWorldSpace( bodySpace );
		}

		Vector3 ApplyResting(const Vector3& impulse)
		{
			if(resting_)
			{
				float dot = impulse.Dot(restingVector);
				if(dot > 0)
				{
					//end resting
					std::cout << "end resting contact in apply resting"<<std::endl;
					resting_ = false;
					restingBodyId = -1;
					return impulse;
				}
				else
				{
					Vector3 out(impulse);
					out.Normalize();
					out = impulse + dot * out;
					return out;
				}
			}
			else
			{
				return impulse;
			}
		}

		virtual void CopyTo(RigidBody* target)
        {  
            target->transform_ = transform_;
            target->velocity_ = velocity_;
            target->inverseMass_ = inverseMass_;
            target->inertia_ = inertia_;
			target->inverseInertia_ = inverseInertia_;
            target->angularVelocity_ = angularVelocity_;
			target->resting_ = resting_;
			target->restingBodyId = restingBodyId;
			target->restingVector = restingVector;
			target->mobile_ = mobile_;
			target->id = id;
        }
        virtual void Render()=0;
		bool Mobile()
		{
			return (inverseMass_ != 0);
		}

		virtual void Print(std::ostream& out) const = 0;

		bool CanCollide(RigidBody* other)
		{
			if(!mobile_ && !other->mobile_) return false;
			if(resting_ && restingBodyId == other->id) return false;
			return true;
		}

		bool Resting()
		{
			return resting_;
		}

		bool BeginRestingContact(RigidBody* other,const Vector3& normal)
		{
			if(!mobile_) return false;
			double mag = fabs((other->Velocity() - Velocity()).Dot(normal));
			std::cout << "Resting contact with mag of "<<mag<<std::endl;
			if(mag < RESTING_CONTACT_EPSILON && other->resting_)
			{
				std::cout << "Begin resting contact!"<<std::endl;
				std::cout<<normal<<std::endl;
				restingBodyId = other->id;
				restingVector = normal;
				resting_ = true;
				velocity_ = ApplyResting(velocity_);
				velocity_.Set(0,0,0);
				angularVelocity_.Set(0,0,0);
				return true;
			}
			return false;
		}

		bool EndRestingContact(RigidBody* other)
		{
			if(!mobile_) return false;
			if(!resting_) return false;
			if(fabs((other->Velocity() - Velocity()).Dot(restingVector)) >= RESTING_CONTACT_EPSILON)
			{
				std::cout<<"End resting contact normal"<<std::endl;
				restingBodyId = -1;
				resting_ = false;
				return true;
			}
			return false;
		}

	protected:
		Transform transform_;
		Vector3 velocity_;
		float inverseMass_;
		float restitution_;
		Matrix33 inertia_;
		Matrix33 inverseInertia_;
		Vector3 angularVelocity_;
		bool mobile_;
		bool resting_;

		int restingBodyId;
		Vector3 restingVector;

	};

	inline std::ostream& operator<<(std::ostream& out,RigidBody& body)
	{
		body.Print(out);
		return out;
	}

} // namespace physics

#endif