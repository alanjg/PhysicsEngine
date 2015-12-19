#ifndef ROTATION_H
#define ROTATION_H
#include "Vector3.h"
#include "Matrix44.h"
#include <iosfwd>

namespace physics 
{

	class Rotationf
	{
	public:
		Rotationf();
		Rotationf(const Vector3f& axis,float angle);
		Rotationf(float q0,float q1,float q2,float q3);

		void Set(const Vector3f& axis,float angle);
		void Set(float q0,float q1,float q2,float q3);
		void Set(const Vector3f& from,const Vector3f& to);

		void Get(Vector3f& axis,float& angle) const;
		Vector3f GetAxis() const;
		float GetAngle() const;

		static Rotationf Slerp(const Rotationf& r0,const Rotationf& r1,float t);
		Rotationf& Invert();
		Rotationf Inverse() const;

		void ScaleAngle(float scale);
		static Rotationf identity() { return Rotationf(0,0,0,1); }

		Rotationf& operator *=(const Rotationf& rhs);
		Vector3f Rotate(const Vector3f& vec) const;
		Vector3f InverseRotate(const Vector3f& vec) const;
	private:
		//not precisely axis and angle, but rather: 
		Vector3f axis_; //sin(angle/2)*axis
		float angle_; //cos(angle/2)
		
		float Norm();
		void Normalize();

		friend Rotationf operator*(const Rotationf& lhs,const Rotationf& rhs);
	};
	
	Rotationf operator*(const Rotationf& lhs,const Rotationf& rhs);
	std::ostream& operator<<(std::ostream& out,const Rotationf& rot);

	class Rotationd
	{
	public:
		Rotationd();
		Rotationd(const Vector3d& axis,double angle);
		Rotationd(double q0,double q1,double q2,double q3);

		void Set(const Vector3d& axis,double angle);
		void Set(double q0,double q1,double q2,double q3);
		void Set(const Vector3d& from,const Vector3d& to);

		void Get(Vector3d& axis,double& angle) const;
		Vector3d GetAxis() const;
		double GetAngle() const;

		static Rotationd Slerp(const Rotationd& r0,const Rotationd& r1,double t);
		Rotationd& Invert();
		Rotationd Inverse() const;

		void ScaleAngle(double scale);
		static Rotationd identity() { return Rotationd(0,0,0,1); }

		Rotationd& operator *=(const Rotationd& rhs);
		Vector3d Rotate(const Vector3d& vec) const;
		Vector3d InverseRotate(const Vector3d& vec) const;

	private:
		//not precisely axis and angle, but rather: 
		Vector3d axis_; //sin(angle/2)*axis
		double angle_; //cos(angle/2)
		
		double Norm();
		void Normalize();

		friend Rotationd operator*(const Rotationd& lhs,const Rotationd& rhs);
	};
	
	Rotationd operator*(const Rotationd& lhs,const Rotationd& rhs);
	std::ostream& operator<<(std::ostream& out,const Rotationd& rot);

	typedef Rotationf Rotation;
}
#endif