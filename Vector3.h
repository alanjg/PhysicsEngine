#ifndef VECTOR3_H
#define VECTOR3_H

#include "mathdefs.h"
#include <cmath>
#include <iosfwd>

namespace physics
{
	class Vector3f
	{
	public:
		Vector3f()
		{
		}
		Vector3f(float x,float y,float z)
		{
			v_[0] = x;
			v_[1] = y;
			v_[2] = z;
		}
		Vector3f(float* v)
		{
			v_[0] = v[0];
			v_[1] = v[1];
			v_[2] = v[2];
		}
		void Set(float *v)
		{
			v_[0] = v[0];
			v_[1] = v[1];
			v_[2] = v[2];
		}
		void Set(float x,float y,float z)
		{
			v_[0] = x;
			v_[1] = y;
			v_[2] = z;
		}
		const float& operator[](int index) const
		{
			return v_[index];
		}
		float& operator[](int index)
		{
			return v_[index];
		}
		Vector3f& operator+=(const Vector3f& rhs)
		{
			v_[0] += rhs.v_[0];
			v_[1] += rhs.v_[1];
			v_[2] += rhs.v_[2];
			return *this;
		}
		Vector3f operator+(const Vector3f& rhs) const
		{
			return Vector3f(*this) += rhs;
		}
		Vector3f& operator-=(const Vector3f& rhs)
		{
			v_[0] -= rhs.v_[0];
			v_[1] -= rhs.v_[1];
			v_[2] -= rhs.v_[2];
			return *this;
		}
		Vector3f operator-(const Vector3f& rhs) const
		{
			return Vector3f(*this) -= rhs;
		}
		Vector3f& operator*=(float rhs)
		{
			v_[0] *= rhs;
			v_[1] *= rhs;
			v_[2] *= rhs;
			return *this;
		}
		friend Vector3f operator*(const Vector3f& lhs, float rhs);
		friend Vector3f operator*(float lhs, const Vector3f& rhs);
		Vector3f& operator/=(float rhs)
		{
			v_[0] /= rhs;
			v_[1] /= rhs;
			v_[2] /= rhs;
			return *this;
		}
		Vector3f operator/(float rhs) const
		{		
			return Vector3f(*this) /= rhs;
		}
		Vector3f operator-() const
		{
			return Vector3f(*this) *= -1;
		}
		float Dot(const Vector3f& rhs) const
		{
			return v_[0]*rhs.v_[0] + v_[1]*rhs.v_[1] + v_[2]*rhs.v_[2];
		}
		Vector3f Cross(const Vector3f& rhs) const
		{
			return Vector3f(v_[1]*rhs.v_[2]-v_[2]*rhs.v_[1],
							v_[2]*rhs.v_[0]-v_[0]*rhs.v_[2],
							v_[0]*rhs.v_[1]-v_[1]*rhs.v_[0]);
		}
		float Magnitude() const 
		{
			return std::sqrtf(MagnitudeSquared());
		}
		float MagnitudeSquared() const
		{
			return Dot(*this);
		}
        float Normalize()
        {
            float mag = Magnitude();
			if(mag==0)
				return 0;

            v_[0] /= mag;
            v_[1] /= mag;
            v_[2] /= mag;
			return mag;
        }
        const float* pointer()
        {
            return &v_[0]; 
        }
		private:
			float v_[3];
	};
	std::ostream& operator<<(std::ostream& out,const Vector3f& v);
	inline bool IsEqual(const Vector3f& lhs,const Vector3f& rhs)
	{
		return IsEqual(lhs[0],rhs[0]) && IsEqual(lhs[1],rhs[1]) && IsEqual(lhs[2],rhs[2]);
	}

	class Vector3d
	{
	public:
		Vector3d()
		{
		}
		Vector3d(double x,double y,double z)
		{
			v_[0] = x;
			v_[1] = y;
			v_[2] = z;
		}
		Vector3d(double* v)
		{
			v_[0] = v[0];
			v_[1] = v[1];
			v_[2] = v[2];
		}
		void Set(double *v)
		{
			v_[0] = v[0];
			v_[1] = v[1];
			v_[2] = v[2];
		}
		void Set(double x,double y,double z)
		{
			v_[0] = x;
			v_[1] = y;
			v_[2] = z;
		}
		const double& operator[](int index) const
		{
			return v_[index];
		}
		double& operator[](int index)
		{
			return v_[index];
		}
		Vector3d& operator+=(const Vector3d& rhs)
		{
			v_[0] += rhs.v_[0];
			v_[1] += rhs.v_[1];
			v_[2] += rhs.v_[2];
			return *this;
		}
		Vector3d operator+(const Vector3d& rhs) const
		{
			return Vector3d(*this) += rhs;
		}
		Vector3d& operator-=(const Vector3d& rhs)
		{
			v_[0] -= rhs.v_[0];
			v_[1] -= rhs.v_[1];
			v_[2] -= rhs.v_[2];
			return *this;
		}
		Vector3d operator-(const Vector3d& rhs) const
		{
			return Vector3d(*this) -= rhs;
		}
		Vector3d& operator*=(double rhs)
		{
			v_[0] *= rhs;
			v_[1] *= rhs;
			v_[2] *= rhs;
			return *this;
		}
		friend Vector3d operator*(const Vector3d& lhs, double rhs);
		friend Vector3d operator*(double lhs, const Vector3d& rhs);
		Vector3d& operator/=(double rhs)
		{
			v_[0] /= rhs;
			v_[1] /= rhs;
			v_[2] /= rhs;
			return *this;
		}
		Vector3d operator/(double rhs) const
		{		
			return Vector3d(*this) /= rhs;
		}
		Vector3d operator-() const
		{
			return Vector3d(*this) *= -1;
		}
		double Dot(const Vector3d& rhs) const
		{
			return v_[0]*rhs.v_[0] + v_[1]*rhs.v_[1] + v_[2]*rhs.v_[2];
		}
		Vector3d Cross(const Vector3d& rhs) const
		{
			return Vector3d(v_[1]*rhs.v_[2]-v_[2]*rhs.v_[1],
							v_[2]*rhs.v_[0]-v_[0]*rhs.v_[2],
							v_[0]*rhs.v_[1]-v_[1]*rhs.v_[0]);
		}
		double Magnitude() const 
		{
			return std::sqrt(MagnitudeSquared());
		}
		double MagnitudeSquared() const
		{
			return Dot(*this);
		}
        double Normalize()
        {
            double mag = Magnitude();
			if(mag==0)
				return 0;

            v_[0] /= mag;
            v_[1] /= mag;
            v_[2] /= mag;
			return mag;
        }
        const double* pointer()
        {
            return &v_[0]; 
        }
		private:
			double v_[3];
	};
	std::ostream& operator<<(std::ostream& out,const Vector3d& v);
	
	inline bool IsEqual(const Vector3d& lhs,const Vector3d& rhs)
	{
		return IsEqual(lhs[0],rhs[0]) && IsEqual(lhs[1],rhs[1]) && IsEqual(lhs[2],rhs[2]);
	}

#ifdef DOUBLE_PRECISION
	typedef Vector3d Vector3;
#else
	typedef Vector3f Vector3;
#endif

}

#endif