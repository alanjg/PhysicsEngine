#ifndef VECTOR2_H
#define VECTOR2_H
#include "mathdefs.h"
#include <cmath>
#include <iosfwd>

namespace physics
{
	class Vector2f
	{
	public:
		Vector2f()
		{
		}
		Vector2f(float x,float y)
		{
			v_[0] = x;
			v_[1] = y;
		}
		Vector2f(float* v)
		{
			v_[0] = v[0];
			v_[1] = v[1];
		}
		void Set(float *v)
		{
			v_[0] = v[0];
			v_[1] = v[1];
		}
		void Set(float x,float y)
		{
			v_[0] = x;
			v_[1] = y;
		}
		const float& operator[](int index) const
		{
			return v_[index];
		}
		float& operator[](int index)
		{
			return v_[index];
		}
		Vector2f& operator+=(const Vector2f& rhs)
		{
			v_[0] += rhs.v_[0];
			v_[1] += rhs.v_[1];
			return *this;
		}
		Vector2f operator+(const Vector2f& rhs) const
		{
			return Vector2f(*this) += rhs;
		}
		Vector2f& operator-=(const Vector2f& rhs)
		{
			v_[0] -= rhs.v_[0];
			v_[1] -= rhs.v_[1];
			return *this;
		}
		Vector2f operator-(const Vector2f& rhs) const
		{
			return Vector2f(*this) -= rhs;
		}
		Vector2f& operator*=(float rhs)
		{
			v_[0] *= rhs;
			v_[1] *= rhs;
			return *this;
		}
		friend Vector2f operator*(const Vector2f& lhs, float rhs);
		friend Vector2f operator*(float lhs, const Vector2f& rhs);
		Vector2f& operator/=(float rhs)
		{
			v_[0] /= rhs;
			v_[1] /= rhs;
			return *this;
		}
		Vector2f operator/(float rhs) const
		{		
			return Vector2f(*this) /= rhs;
		}
		Vector2f operator-() const
		{
			return Vector2f(*this) *= -1;
		}
		float Dot(const Vector2f& rhs) const
		{
			return v_[0]*rhs.v_[0] + v_[1]*rhs.v_[1];
		}
		float Cross(const Vector2f& rhs) const
		{
			return v_[0]*rhs.v_[1]-v_[1]*rhs.v_[0];
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
			return mag;
        }
        const float* pointer()
        {
            return &v_[0]; 
        }
		private:
			float v_[2];
	};
	std::ostream& operator<<(std::ostream& out,const Vector2f& v);
	inline bool IsEqual(const Vector2f& lhs,const Vector2f& rhs)
	{
		return IsEqual(lhs[0],rhs[0]) && IsEqual(lhs[1],rhs[1]);
	}

	class Vector2d
	{
	public:
		Vector2d()
		{
		}
		Vector2d(double x,double y)
		{
			v_[0] = x;
			v_[1] = y;
		}
		Vector2d(double* v)
		{
			v_[0] = v[0];
			v_[1] = v[1];
		}
		void Set(double *v)
		{
			v_[0] = v[0];
			v_[1] = v[1];
		}
		void Set(double x,double y)
		{
			v_[0] = x;
			v_[1] = y;
		}
		const double& operator[](int index) const
		{
			return v_[index];
		}
		double& operator[](int index)
		{
			return v_[index];
		}
		Vector2d& operator+=(const Vector2d& rhs)
		{
			v_[0] += rhs.v_[0];
			v_[1] += rhs.v_[1];
			return *this;
		}
		Vector2d operator+(const Vector2d& rhs) const
		{
			return Vector2d(*this) += rhs;
		}
		Vector2d& operator-=(const Vector2d& rhs)
		{
			v_[0] -= rhs.v_[0];
			v_[1] -= rhs.v_[1];
			return *this;
		}
		Vector2d operator-(const Vector2d& rhs) const
		{
			return Vector2d(*this) -= rhs;
		}
		Vector2d& operator*=(double rhs)
		{
			v_[0] *= rhs;
			v_[1] *= rhs;
			return *this;
		}
		friend Vector2d operator*(const Vector2d& lhs, double rhs);
		friend Vector2d operator*(double lhs, const Vector2d& rhs);
		Vector2d& operator/=(double rhs)
		{
			v_[0] /= rhs;
			v_[1] /= rhs;
			return *this;
		}
		Vector2d operator/(double rhs) const
		{		
			return Vector2d(*this) /= rhs;
		}
		Vector2d operator-() const
		{
			return Vector2d(*this) *= -1;
		}
		double Dot(const Vector2d& rhs) const
		{
			return v_[0]*rhs.v_[0] + v_[1]*rhs.v_[1];
		}
		double Cross(const Vector2d& rhs) const
		{
			return v_[0]*rhs.v_[1]-v_[1]*rhs.v_[0];
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
			return mag;
        }
        const double* pointer()
        {
            return &v_[0]; 
        }
		private:
			double v_[2];
	};
	std::ostream& operator<<(std::ostream& out,const Vector2d& v);
	inline bool IsEqual(const Vector2d& lhs,const Vector2d& rhs)
	{
		return IsEqual(lhs[0],rhs[0]) && IsEqual(lhs[1],rhs[1]);
	}

#ifdef DOUBLE_PRECISION
	typedef Vector2d Vector2;
#else
	typedef Vector2f Vector2;
#endif

}
#endif