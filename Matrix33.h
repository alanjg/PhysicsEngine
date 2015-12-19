#ifndef MATRIX33_H
#define MATRIX33_H

#include "Vector3.h"

namespace physics
{

	class Matrix33f
	{
	public:
		Matrix33f()
		{
		}

		Matrix33f(const Vector3f& rowX,const Vector3f& rowY,const Vector3f& rowZ)
		{
			rows[0] = rowX;
			rows[1] = rowY;
			rows[2] = rowZ;
		}
		Matrix33f(float m00,float m01,float m02,float m10,float m11,float m12,float m20,float m21,float m22)
		{
			rows[0][0] = m00;
			rows[0][1] = m01;
			rows[0][2] = m02;
			rows[1][0] = m10;
			rows[1][1] = m11;
			rows[1][2] = m12;
			rows[2][0] = m20;
			rows[2][1] = m21;
			rows[2][2] = m22;
		}
		Matrix33f(float* m)
		{
			rows[0][0] = m[0];
			rows[0][1] = m[1];
			rows[0][2] = m[2];
			rows[1][0] = m[3];
			rows[1][1] = m[4];
			rows[1][2] = m[5];
			rows[2][0] = m[6];
			rows[2][1] = m[7];
			rows[2][2] = m[8];
		}
		void Set(float m00,float m01,float m02,float m10,float m11,float m12,float m20,float m21,float m22)
		{
			rows[0][0] = m00;
			rows[0][1] = m01;
			rows[0][2] = m02;
			rows[1][0] = m10;
			rows[1][1] = m11;
			rows[1][2] = m12;
			rows[2][0] = m20;
			rows[2][1] = m21;
			rows[2][2] = m22;
		}
		void Set(const Vector3f& rowX,const Vector3f& rowY,const Vector3f& rowZ)
		{
			rows[0] = rowX;
			rows[1] = rowY;
			rows[2] = rowZ;
		}
		void Set(float* m)
		{
			rows[0][0] = m[0];
			rows[0][1] = m[1];
			rows[0][2] = m[2];
			rows[1][0] = m[3];
			rows[1][1] = m[4];
			rows[1][2] = m[5];
			rows[2][0] = m[6];
			rows[2][1] = m[7];
			rows[2][2] = m[8];
		}

		float Determinant() const
		{
			return rows[0][0] * rows[1][1] * rows[2][2] - rows[0][0] * rows[2][1] * rows[1][2] + rows[1][0] * rows[2][1] * rows[0][2] - 
				rows[1][0] * rows[0][1] * rows[2][2] + rows[2][0] * rows[0][1] * rows[1][2] - rows[2][0] * rows[1][1] * rows[0][2];
		}

		void MakeIdentity()
		{
			rows[0][0] = rows[1][1] = rows[2][2] = 1;
			rows[0][1] = rows[0][2] = rows[1][0] = rows[1][2] = rows[2][0] = rows[2][1] = 0;
		}

		Matrix33f Transpose() const
		{
			return Matrix33f(rows[0][0],rows[1][0],rows[2][0],rows[0][1],rows[1][1],rows[2][1],rows[0][2],rows[1][2],rows[2][2]);
		}

		Matrix33f Inverse() const
		{
			float det = Determinant();
			if(IsZero(det))
				det = 1;

			return Matrix33f( (rows[1][1]*rows[2][2] - rows[1][2]*rows[2][1]),
							-(rows[0][1]*rows[2][2] - rows[0][2]*rows[2][1]),
								(rows[0][1]*rows[1][2] - rows[0][2]*rows[1][1]),
							-(rows[2][0]*rows[2][2] - rows[2][1]*rows[2][0]),
								(rows[0][0]*rows[2][2] - rows[0][2]*rows[2][0]),
							-(rows[0][0]*rows[1][2] - rows[0][2]*rows[1][0]),
								(rows[1][0]*rows[2][1] - rows[1][1]*rows[2][0]),
							-(rows[0][0]*rows[2][1] - rows[0][1]*rows[2][0]),
								(rows[0][0]*rows[1][1] - rows[0][1]*rows[1][0]))/=det;
		}

		void MakeRotate(float radians,const Vector3f& axis_)
		{
			Vector3f axis = axis_;
			axis.Normalize();

			float c=cosf(radians);
			float s=sinf(radians);
			float t=1-c;
			rows[0][0]=t*axis[0]*axis[0]+c;
			rows[0][1]=t*axis[0]*axis[1]-s*axis[2];
			rows[0][2]=t*axis[2]*axis[0]+s*axis[1];
			rows[1][0]=t*axis[0]*axis[1]+s*axis[2];
			rows[1][1]=t*axis[1]*axis[1]+c;
			rows[1][2]=t*axis[1]*axis[2]-s*axis[0];
			rows[2][0]=t*axis[2]*axis[0]-s*axis[1];
			rows[2][1]=t*axis[1]*axis[2]+s*axis[0];
			rows[2][2]=t*axis[2]*axis[2]+c;
		}
		void MakeRotate(float radians,float x,float y,float z)
		{
			MakeRotate(radians,Vector3f(x,y,z));
		}

		Matrix33f& operator+=(const Matrix33f& rhs)
		{
			rows[0] += rhs.rows[0];
			rows[1] += rhs.rows[1];
			rows[2] += rhs.rows[2];
			return *this;
		}

		Matrix33f& operator-=(const Matrix33f& rhs)
		{
			rows[0] -= rhs.rows[0];
			rows[1] -= rhs.rows[1];
			rows[2] -= rhs.rows[2];
			return *this;
		}
		
		Matrix33f& operator*=(const Matrix33f& rhs)
		{
			Matrix33f temp;
			for(int i=0;i<3;i++)
			{
				for(int j=0;j<3;j++)
				{
					temp[i][j] = rows[i][0] * rhs.rows[0][j] +
						            rows[i][1] * rhs.rows[1][j] +
						            rows[i][2] * rhs.rows[2][j];
				}
			}
			rows[0] = temp.rows[0];
			rows[1] = temp.rows[1];
			rows[2] = temp.rows[2];
			return *this;
		}

		Matrix33f& operator*=(float rhs)
		{
			rows[0] *= rhs;
			rows[1] *= rhs;
			rows[2] *= rhs;
			return *this;
		}

		Matrix33f& operator/=(float rhs)
		{
			float inverse = 1/rhs;
			rows[0] *= inverse;
			rows[1] *= inverse;
			rows[2] *= inverse;
			return *this;
		}

		Vector3f& operator[](int index) 
		{
			return rows[index];
		}

		const Vector3f& operator[](int index) const
		{
			return rows[index];
		}

		friend Matrix33f operator*(const Matrix33f& lhs,const Matrix33f& rhs);
		friend Matrix33f operator+(const Matrix33f& lhs,const Matrix33f& rhs);
		friend Matrix33f operator-(const Matrix33f& lhs,const Matrix33f& rhs);
		friend Matrix33f operator*(const Matrix33f& lhs,float rhs);
		friend Matrix33f operator*(float lhs,const Matrix33f& rhs);
		friend Matrix33f operator/(const Matrix33f& lhs,float rhs);
		friend Vector3f operator*(const Matrix33f& lhs,const Vector3f& rhs);
		friend Vector3f operator*(const Vector3f& lhs,const Matrix33f& rhs);

	private:
		Vector3f rows[3];
	};

	inline Matrix33f operator*(const Matrix33f& lhs,const Matrix33f& rhs)
	{
		return Matrix33f(lhs) *= rhs;
	}

	inline Matrix33f operator+(const Matrix33f& lhs,const Matrix33f& rhs)
	{
		return Matrix33f(lhs) += rhs;
	}

	inline Matrix33f operator-(const Matrix33f& lhs,const Matrix33f& rhs)
	{
		return Matrix33f(lhs) -= rhs;
	}

	inline Matrix33f operator*(const Matrix33f& lhs,float rhs)
	{
		return Matrix33f(lhs) *= rhs;
	}

	inline Matrix33f operator*(float lhs,const Matrix33f& rhs)
	{
		return Matrix33f(rhs) *= lhs;
	}

	inline Matrix33f operator/(const Matrix33f& lhs,float rhs)
	{
		float recip = 1/rhs;
		return Matrix33f(lhs) *= recip;
	}

	inline Vector3f operator*(const Matrix33f& lhs,const Vector3f& rhs)
	{
		return Vector3f(lhs.rows[0].Dot(rhs),lhs.rows[1].Dot(rhs),lhs.rows[2].Dot(rhs));
	}

	inline Vector3f operator*(const Vector3f& lhs,const Matrix33f& rhs)
	{
		Vector3f a(rhs.rows[0][0],rhs.rows[1][0],rhs.rows[2][0]);
		Vector3f b(rhs.rows[0][1],rhs.rows[1][1],rhs.rows[2][1]);
		Vector3f c(rhs.rows[0][2],rhs.rows[1][2],rhs.rows[2][2]);

		return Vector3f(a.Dot(lhs),b.Dot(lhs),c.Dot(lhs));
	}

	class Matrix33d
	{
	public:
		Matrix33d()
		{
		}

		Matrix33d(const Vector3d& rowX,const Vector3d& rowY,const Vector3d& rowZ)
		{
			rows[0] = rowX;
			rows[1] = rowY;
			rows[2] = rowZ;
		}
		Matrix33d(double m00,double m01,double m02,double m10,double m11,double m12,double m20,double m21,double m22)
		{
			rows[0][0] = m00;
			rows[0][1] = m01;
			rows[0][2] = m02;
			rows[1][0] = m10;
			rows[1][1] = m11;
			rows[1][2] = m12;
			rows[2][0] = m20;
			rows[2][1] = m21;
			rows[2][2] = m22;
		}
		Matrix33d(double* m)
		{
			rows[0][0] = m[0];
			rows[0][1] = m[1];
			rows[0][2] = m[2];
			rows[1][0] = m[3];
			rows[1][1] = m[4];
			rows[1][2] = m[5];
			rows[2][0] = m[6];
			rows[2][1] = m[7];
			rows[2][2] = m[8];
		}
		void Set(double m00,double m01,double m02,double m10,double m11,double m12,double m20,double m21,double m22)
		{
			rows[0][0] = m00;
			rows[0][1] = m01;
			rows[0][2] = m02;
			rows[1][0] = m10;
			rows[1][1] = m11;
			rows[1][2] = m12;
			rows[2][0] = m20;
			rows[2][1] = m21;
			rows[2][2] = m22;
		}
		void Set(const Vector3d& rowX,const Vector3d& rowY,const Vector3d& rowZ)
		{
			rows[0] = rowX;
			rows[1] = rowY;
			rows[2] = rowZ;
		}
		void Set(double* m)
		{
			rows[0][0] = m[0];
			rows[0][1] = m[1];
			rows[0][2] = m[2];
			rows[1][0] = m[3];
			rows[1][1] = m[4];
			rows[1][2] = m[5];
			rows[2][0] = m[6];
			rows[2][1] = m[7];
			rows[2][2] = m[8];
		}

		double Determinant() const
		{
			return rows[0][0] * rows[1][1] * rows[2][2] - rows[0][0] * rows[2][1] * rows[1][2] + rows[1][0] * rows[2][1] * rows[0][2] - 
				rows[1][0] * rows[0][1] * rows[2][2] + rows[2][0] * rows[0][1] * rows[1][2] - rows[2][0] * rows[1][1] * rows[0][2];
		}

		void MakeIdentity()
		{
			rows[0][0] = rows[1][1] = rows[2][2] = 1;
			rows[0][1] = rows[0][2] = rows[1][0] = rows[1][2] = rows[2][0] = rows[2][1] = 0;
		}

		Matrix33d Transpose() const
		{
			return Matrix33d(rows[0][0],rows[1][0],rows[2][0],rows[0][1],rows[1][1],rows[2][1],rows[0][2],rows[1][2],rows[2][2]);
		}

		Matrix33d Inverse() const
		{
			double det = Determinant();
			if(IsZero(det))
				det = 1;

			return Matrix33d( (rows[1][1]*rows[2][2] - rows[1][2]*rows[2][1]),
							-(rows[0][1]*rows[2][2] - rows[0][2]*rows[2][1]),
								(rows[0][1]*rows[1][2] - rows[0][2]*rows[1][1]),
							-(rows[2][0]*rows[2][2] - rows[2][1]*rows[2][0]),
								(rows[0][0]*rows[2][2] - rows[0][2]*rows[2][0]),
							-(rows[0][0]*rows[1][2] - rows[0][2]*rows[1][0]),
								(rows[1][0]*rows[2][1] - rows[1][1]*rows[2][0]),
							-(rows[0][0]*rows[2][1] - rows[0][1]*rows[2][0]),
								(rows[0][0]*rows[1][1] - rows[0][1]*rows[1][0]))/=det;
		}

		Matrix33d& operator+=(const Matrix33d& rhs)
		{
			rows[0] += rhs.rows[0];
			rows[1] += rhs.rows[1];
			rows[2] += rhs.rows[2];
			return *this;
		}

		Matrix33d& operator-=(const Matrix33d& rhs)
		{
			rows[0] -= rhs.rows[0];
			rows[1] -= rhs.rows[1];
			rows[2] -= rhs.rows[2];
			return *this;
		}
		
		Matrix33d& operator*=(const Matrix33d& rhs)
		{
			Matrix33d temp;
			for(int i=0;i<3;i++)
			{
				for(int j=0;j<3;j++)
				{
					temp[i][j] = rows[i][0] * rhs.rows[0][j] +
						            rows[i][1] * rhs.rows[1][j] +
						            rows[i][2] * rhs.rows[2][j];
				}
			}
			rows[0] = temp.rows[0];
			rows[1] = temp.rows[1];
			rows[2] = temp.rows[2];
			return *this;
		}

		Matrix33d& operator*=(double rhs)
		{
			rows[0] *= rhs;
			rows[1] *= rhs;
			rows[2] *= rhs;
			return *this;
		}

		Matrix33d& operator/=(double rhs)
		{
			double inverse = 1/rhs;
			rows[0] *= inverse;
			rows[1] *= inverse;
			rows[2] *= inverse;
			return *this;
		}

		Vector3d& operator[](int index) 
		{
			return rows[index];
		}

		const Vector3d& operator[](int index) const
		{
			return rows[index];
		}

		friend Matrix33d operator*(const Matrix33d& lhs,const Matrix33d& rhs);
		friend Matrix33d operator+(const Matrix33d& lhs,const Matrix33d& rhs);
		friend Matrix33d operator-(const Matrix33d& lhs,const Matrix33d& rhs);
		friend Matrix33d operator*(const Matrix33d& lhs,double rhs);
		friend Matrix33d operator*(double lhs,const Matrix33d& rhs);
		friend Matrix33d operator/(const Matrix33d& lhs,double rhs);
		friend Vector3d operator*(const Matrix33d& lhs,const Vector3d& rhs);
		friend Vector3d operator*(const Vector3d& lhs,const Matrix33d& rhs);

	private:
		Vector3d rows[3];
	};

	inline Matrix33d operator*(const Matrix33d& lhs,const Matrix33d& rhs)
	{
		return Matrix33d(lhs) *= rhs;
	}

	inline Matrix33d operator+(const Matrix33d& lhs,const Matrix33d& rhs)
	{
		return Matrix33d(lhs) += rhs;
	}

	inline Matrix33d operator-(const Matrix33d& lhs,const Matrix33d& rhs)
	{
		return Matrix33d(lhs) -= rhs;
	}

	inline Matrix33d operator*(const Matrix33d& lhs,double rhs)
	{
		return Matrix33d(lhs) *= rhs;
	}

	inline Matrix33d operator*(double lhs,const Matrix33d& rhs)
	{
		return Matrix33d(rhs) *= lhs;
	}

	inline Matrix33d operator/(const Matrix33d& lhs,double rhs)
	{
		double recip = 1/rhs;
		return Matrix33d(lhs) *= recip;
	}

	inline Vector3d operator*(const Matrix33d& lhs,const Vector3d& rhs)
	{
		return Vector3d(lhs.rows[0].Dot(rhs),lhs.rows[1].Dot(rhs),lhs.rows[2].Dot(rhs));
	}

	inline Vector3d operator*(const Vector3d& lhs,const Matrix33d& rhs)
	{
		Vector3d a(rhs.rows[0][0],rhs.rows[1][0],rhs.rows[2][0]);
		Vector3d b(rhs.rows[0][1],rhs.rows[1][1],rhs.rows[2][1]);
		Vector3d c(rhs.rows[0][2],rhs.rows[1][2],rhs.rows[2][2]);

		return Vector3d(a.Dot(lhs),b.Dot(lhs),c.Dot(lhs));
	}

#ifdef DOUBLE_PRECISION
	typedef Matrix33d Matrix33;
#else
	typedef Matrix33f Matrix33;
#endif

} // namespace physics

#endif