#ifndef VECTOR4_H
#define VECTOR4_H
#include <cmath>
#include <iosfwd>
#include "Vector3.h"

namespace physics
{
	class Vector4f
	{
	public:
		Vector4f(float array[4])
		{
			v_[0] = array[0]; 
			v_[1] = array[1];
			v_[2] = array[2];
			v_[3] = array[3];
		}
		Vector4f()
		{
		}
		Vector4f(const Vector3f& vec)
		{
			v_[0] = vec[0];
			v_[1] = vec[1];
			v_[2] = vec[2];
			v_[3] = 1.0f;
		}
		Vector4f(const Vector3f& vec,float w)
		{
			v_[0] = vec[0];
			v_[1] = vec[1];
			v_[2] = vec[2];
			v_[3] = w;
		}
		Vector4f(float x,float y,float z, float w)
		{
			v_[0] = x;
			v_[1] = y;
			v_[2] = z;
			v_[3] = w;
		}
		Vector3f MakeVector3() const
		{
			return Vector3f(v_[0]/v_[3],v_[1]/v_[3],v_[2]/v_[3]);
		}
		Vector3f GetVector3() const
		{
			return Vector3f(v_[0],v_[1],v_[2]);
		}
		void Set(float x,float y,float z,float w)
		{
			v_[0] = x;
			v_[1] = y;
			v_[2] = z;
			v_[3] = w;
		}
		const float& operator[](int index) const
		{
			return v_[index];
		}
		float& operator[](int index)
		{
			return v_[index];
		}
		Vector4f& operator+=(const Vector4f& rhs)
		{
			v_[0] += rhs.v_[0];
			v_[1] += rhs.v_[1];
			v_[2] += rhs.v_[2];
			v_[3] += rhs.v_[3];
			return *this;
		}
		Vector4f operator+(const Vector4f& rhs) const
		{
			return Vector4f(*this) += rhs;
		}
		Vector4f& operator-=(const Vector4f& rhs)
		{
			v_[0] -= rhs.v_[0];
			v_[1] -= rhs.v_[1];
			v_[2] -= rhs.v_[2];
			v_[3] -= rhs.v_[3];
			return *this;
		}
		Vector4f operator-(const Vector4f& rhs) const
		{
			return Vector4f(*this) -= rhs;
		}
		Vector4f& operator*=(float rhs)
		{
			v_[0] *= rhs;
			v_[1] *= rhs;
			v_[2] *= rhs;
			v_[3] *= rhs;
			return *this;
		}
		friend Vector4f operator*(const Vector4f& lhs, float rhs);
		friend Vector4f operator*(float lhs, const Vector4f& rhs);
		Vector4f& operator/=(float rhs)
		{
			v_[0] /= rhs;
			v_[1] /= rhs;
			v_[2] /= rhs;
			v_[3] /= rhs;
			return *this;
		}
		Vector4f operator/(float rhs) const
		{		
			return Vector4f(*this) /= rhs;
		}
		Vector4f operator-() const
		{
			return Vector4f(*this) *= -1;
		}
		float Dot(const Vector4f& rhs) const
		{
			return v_[0]*rhs.v_[0] + v_[1]*rhs.v_[1] + v_[2]*rhs.v_[2] + v_[3]*rhs.v_[3];
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
			v_[3] /= mag;
			return mag;
        }
        const float* pointer()
        {
            return &v_[0]; 
        }
		private:
			float v_[4];
	};
	std::ostream& operator<<(std::ostream& out,const Vector4f& v);
	inline bool IsEqual(const Vector4f& lhs,const Vector4f& rhs)
	{
		return IsEqual(lhs[0],rhs[0]) && IsEqual(lhs[1],rhs[1]) && IsEqual(lhs[2],rhs[2]) && IsEqual(lhs[3],rhs[3]);
	}

	class Vector4d
	{
	public:
		Vector4d(double array[4])
		{
			v_[0] = array[0]; 
			v_[1] = array[1];
			v_[2] = array[2];
			v_[3] = array[3];
		}
		Vector4d()
		{
		}
		Vector4d(const Vector3d& vec)
		{
			v_[0] = vec[0];
			v_[1] = vec[1];
			v_[2] = vec[2];
			v_[3] = 1.0f;
		}
		Vector4d(const Vector3d& vec,double w)
		{
			v_[0] = vec[0];
			v_[1] = vec[1];
			v_[2] = vec[2];
			v_[3] = w;
		}
		Vector4d(double x,double y,double z, double w)
		{
			v_[0] = x;
			v_[1] = y;
			v_[2] = z;
			v_[3] = w;
		}
		Vector3d MakeVector3() const
		{
			return Vector3d(v_[0]/v_[3],v_[1]/v_[3],v_[2]/v_[3]);
		}
		Vector3d GetVector3() const
		{
			return Vector3d(v_[0],v_[1],v_[2]);
		}
		void Set(double x,double y,double z,double w)
		{
			v_[0] = x;
			v_[1] = y;
			v_[2] = z;
			v_[3] = w;
		}
		const double& operator[](int index) const
		{
			return v_[index];
		}
		double& operator[](int index)
		{
			return v_[index];
		}
		Vector4d& operator+=(const Vector4d& rhs)
		{
			v_[0] += rhs.v_[0];
			v_[1] += rhs.v_[1];
			v_[2] += rhs.v_[2];
			v_[3] += rhs.v_[3];
			return *this;
		}
		Vector4d operator+(const Vector4d& rhs) const
		{
			return Vector4d(*this) += rhs;
		}
		Vector4d& operator-=(const Vector4d& rhs)
		{
			v_[0] -= rhs.v_[0];
			v_[1] -= rhs.v_[1];
			v_[2] -= rhs.v_[2];
			v_[3] -= rhs.v_[3];
			return *this;
		}
		Vector4d operator-(const Vector4d& rhs) const
		{
			return Vector4d(*this) -= rhs;
		}
		Vector4d& operator*=(double rhs)
		{
			v_[0] *= rhs;
			v_[1] *= rhs;
			v_[2] *= rhs;
			v_[3] *= rhs;
			return *this;
		}
		friend Vector4d operator*(const Vector4d& lhs, double rhs);
		friend Vector4d operator*(double lhs, const Vector4d& rhs);
		Vector4d& operator/=(double rhs)
		{
			v_[0] /= rhs;
			v_[1] /= rhs;
			v_[2] /= rhs;
			v_[3] /= rhs;
			return *this;
		}
		Vector4d operator/(double rhs) const
		{		
			return Vector4d(*this) /= rhs;
		}
		Vector4d operator-() const
		{
			return Vector4d(*this) *= -1;
		}
		double Dot(const Vector4d& rhs) const
		{
			return v_[0]*rhs.v_[0] + v_[1]*rhs.v_[1] + v_[2]*rhs.v_[2] + v_[3]*rhs.v_[3];
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
			v_[3] /= mag;
			return mag;
        }
        const double* pointer()
        {
            return &v_[0]; 
        }
		private:
			double v_[4];
	};
	std::ostream& operator<<(std::ostream& out,const Vector4d& v);
	inline bool IsEqual(const Vector4d& lhs,const Vector4d& rhs)
	{
		return IsEqual(lhs[0],rhs[0]) && IsEqual(lhs[1],rhs[1]) && IsEqual(lhs[2],rhs[2]) && IsEqual(lhs[3],rhs[3]);
	}

	typedef Vector4f Vector4;

}
#endif
