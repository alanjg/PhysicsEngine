#include "Matrix44.h"
#include "Vector3.h"
#include "mathdefs.h"
#include <iostream>

namespace {
void swap(float& a,float& b)
{
	float temp = a;
	a = b;
	b = temp;
}
void swap(double& a,double& b)
{
	double temp = a;
	a = b;
	b = temp;
}
}
namespace physics
{
	Matrix44f::Matrix44f()
	{
	}

	Matrix44f::Matrix44f(float m[4][4])
	{
		v[0][0]=m[0][0]; v[0][1]=m[0][1]; v[0][2]=m[0][2]; v[0][3]=m[0][3];
		v[1][0]=m[1][0]; v[1][1]=m[1][1]; v[1][2]=m[1][2]; v[1][3]=m[1][3];
		v[2][0]=m[2][0]; v[2][1]=m[2][1]; v[2][2]=m[2][2]; v[2][3]=m[2][3];
		v[3][0]=m[3][0]; v[3][1]=m[3][1]; v[3][2]=m[3][2]; v[3][3]=m[3][3];
	}

	Matrix44f::Matrix44f(float m[16])
	{
		v[0][0]=m[0]; v[0][1]=m[1]; v[0][2]=m[2]; v[0][3]=m[3];
		v[1][0]=m[4]; v[1][1]=m[5]; v[1][2]=m[6]; v[1][3]=m[7];
		v[2][0]=m[8]; v[2][1]=m[9]; v[2][2]=m[10]; v[2][3]=m[11];
		v[3][0]=m[12]; v[3][1]=m[13]; v[3][2]=m[14]; v[3][3]=m[15];
	}

	Matrix44f::Matrix44f(Vector4f vec[4])
	{
		v[0] = vec[0];
		v[1] = vec[1];
		v[2] = vec[2];
		v[3] = vec[3];
	}

	Matrix44f::Matrix44f(const Matrix44f& m)
	{
		v[0] = m[0];
		v[1] = m[1];
		v[2] = m[2];
		v[3] = m[3];
	}

	Matrix44f Matrix44f::operator*(const Matrix44f& c) const
	{
		return Matrix44f(*this) *= c;
	}

	Vector4f Matrix44f::operator*(const Vector4f& c) const
	{
		Vector4f result;

		for(int i=0;i<4;i++)
		{
			result[i] = v[i].Dot(c);
		}
		return result;

	}

	Matrix44f Matrix44f::operator*(float f) const
	{
		Matrix44f result(*this);

		result[0] *= f;
		result[1] *= f;
		result[2] *= f;
		result[3] *= f;

		return result;
	}

	Matrix44f& Matrix44f::operator*=(const Matrix44f& c)
	{
		Matrix44f temp(*this);
		for(int i=0;i<4;i++)
		{		
			for(int j=0;j<4;j++)
			{
				float t=0;
				for(int k=0;k<4;k++)
					t += temp[i][k] * c[k][j];
				v[i][j] = t;
			}
		}
		return *this;
	}

	bool Matrix44f::operator==(const Matrix44f& m) const
	{
		return v[0][0]==m[0][0]&& v[0][1]==m[0][1]&& v[0][2]==m[0][2]&& v[0][3]==m[0][3]&&
		v[1][0]==m[1][0]&& v[1][1]==m[1][1]&& v[1][2]==m[1][2]&& v[1][3]==m[1][3]&&
		v[2][0]==m[2][0]&& v[2][1]==m[2][1]&& v[2][2]==m[2][2]&& v[2][3]==m[2][3]&&
		v[3][0]==m[3][0]&& v[3][1]==m[3][1]&& v[3][2]==m[3][2]&& v[3][3]==m[3][3];

	}

	void Matrix44f::MakeIdentity()
	{
		v[0][0]=1; v[0][1]=0; v[0][2]=0; v[0][3]=0;
		v[1][0]=0; v[1][1]=1; v[1][2]=0; v[1][3]=0;
		v[2][0]=0; v[2][1]=0; v[2][2]=1; v[2][3]=0;
		v[3][0]=0; v[3][1]=0; v[3][2]=0; v[3][3]=1;
	}

	void Matrix44f::MakeTranslate(const Vector3f& c)
	{
		v[0][0]=1; v[0][1]=0; v[0][2]=0; v[0][3]=c[0];
		v[1][0]=0; v[1][1]=1; v[1][2]=0; v[1][3]=c[1];
		v[2][0]=0; v[2][1]=0; v[2][2]=1; v[2][3]=c[2];
		v[3][0]=0; v[3][1]=0; v[3][2]=0; v[3][3]=1;
	}

	void Matrix44f::MakeTranslate(float x,float y,float z)
	{
		v[0][0]=1; v[0][1]=0; v[0][2]=0; v[0][3]=x;
		v[1][0]=0; v[1][1]=1; v[1][2]=0; v[1][3]=y;
		v[2][0]=0; v[2][1]=0; v[2][2]=1; v[2][3]=z;
		v[3][0]=0; v[3][1]=0; v[3][2]=0; v[3][3]=1;
	}

	void Matrix44f::MakeRotate(float radians,const Vector3f& axis_)
	{
		Vector3f axis = axis_;
		axis.Normalize();

		float c=cosf(radians);
		float s=sinf(radians);
		float t=1-c;
		v[0][0]=t*axis[0]*axis[0]+c;
		v[0][1]=t*axis[0]*axis[1]-s*axis[2];
		v[0][2]=t*axis[2]*axis[0]+s*axis[1];
		v[0][3]=0;
		v[1][0]=t*axis[0]*axis[1]+s*axis[2];
		v[1][1]=t*axis[1]*axis[1]+c;
		v[1][2]=t*axis[1]*axis[2]-s*axis[0];
		v[1][3]=0;
		v[2][0]=t*axis[2]*axis[0]-s*axis[1];
		v[2][1]=t*axis[1]*axis[2]+s*axis[0];
		v[2][2]=t*axis[2]*axis[2]+c;
		v[2][3]=0;
		v[3][0]=0;
		v[3][1]=0;
		v[3][2]=0;
		v[3][3]=1;
	}
	void Matrix44f::MakeRotate(float radians,float x,float y,float z)
	{	
		MakeRotate(radians,Vector3f(x,y,z));
	}
	
	void Matrix44f::MakeScale(const Vector3f& c)
	{
		v[0][0]=c[0];	v[0][1]=0;		v[0][2]=0;		v[0][3]=0;
		v[1][0]=0;		v[1][1]=c[1];	v[1][2]=0;		v[1][3]=0;
		v[2][0]=0;		v[2][1]=0;		v[2][2]=c[2];	v[2][3]=0;
		v[3][0]=0;		v[3][1]=0;		v[3][2]=0;		v[3][3]=1;
	}

	void Matrix44f::MakeScale(float x,float y,float z)
	{
		v[0][0]=x;	v[0][1]=0;	v[0][2]=0;	v[0][3]=0;
		v[1][0]=0;	v[1][1]=y;	v[1][2]=0;	v[1][3]=0;
		v[2][0]=0;	v[2][1]=0;	v[2][2]=z;	v[2][3]=0;
		v[3][0]=0;	v[3][1]=0;	v[3][2]=0;	v[3][3]=1;
	}

	float Matrix44f::Det4() const
	{
		return v[0][0]*Det3(1,2,3,1,2,3) - v[0][1]*Det3(1,2,3,0,2,3) + v[0][2]*Det3(1,2,3,0,1,3) - v[0][3]*Det3(1,2,3,0,1,2);
	}
	
	float Matrix44f::Det3(int i1,int i2,int i3,int j1,int j2,int j3) const
	{
		return v[i1][j1]*Det2(i2,i3,j2,j3)-v[i1][j2]*Det2(i2,i3,j1,j3)+v[i1][j3]*Det2(i2,i3,j1,j2);
	}

	float Matrix44f::Det2(int i1,int i2,int j1,int j2) const
	{
		return v[i1][j1]*v[i2][j2]-v[i1][j2]*v[i2][j1];
	}

	void Matrix44f::Invert()
	{
		float det = Det4();
		if(det == 0)
		{
			std::cerr<<"determinant is zero!!"<<std::endl;
			return;
		}
		float inverseDet = 1.0f/det;
		float d[4][4][4][4];
		for(int i1=0;i1<4;i1++)
			for(int i2=0;i2<4;i2++)
				for(int j1=0;j1<4;j1++)
					for(int j2=0;j2<4;j2++)
						d[i1][i2][j1][j2] = Det2(i1,i2,j1,j2);
		Matrix44f result;
		result[0][0] = v[1][1]*d[2][3][2][3] - v[1][2]*d[2][3][1][3] + v[1][3]*d[2][3][1][2];
		result[0][1] = v[1][0]*d[2][3][2][3] - v[1][2]*d[2][3][0][3] + v[1][3]*d[2][3][0][2];
		result[0][2] = v[1][0]*d[2][3][1][3] - v[1][1]*d[2][3][0][3] + v[1][3]*d[2][3][0][1];
		result[0][3] = v[1][0]*d[2][3][1][2] - v[1][1]*d[2][3][0][2] + v[1][2]*d[2][3][0][1];
		
		result[1][0] = v[0][1]*d[2][3][2][3] - v[0][2]*d[2][3][1][3] + v[0][3]*d[2][3][1][2];
		result[1][1] = v[0][0]*d[2][3][2][3] - v[0][2]*d[2][3][0][3] + v[0][3]*d[2][3][0][2];
		result[1][2] = v[0][0]*d[2][3][1][3] - v[0][1]*d[2][3][0][3] + v[0][3]*d[2][3][0][1];
		result[1][3] = v[0][0]*d[2][3][1][2] - v[0][1]*d[2][3][0][2] + v[0][2]*d[2][3][0][1];
		
		result[2][0] = v[0][1]*d[1][3][2][3] - v[0][2]*d[1][3][1][3] + v[0][3]*d[1][3][1][2];
		result[2][1] = v[0][0]*d[1][3][2][3] - v[0][2]*d[1][3][0][3] + v[0][3]*d[1][3][0][2];
		result[2][2] = v[0][0]*d[1][3][1][3] - v[0][1]*d[1][3][0][3] + v[0][3]*d[1][3][0][1];
		result[2][3] = v[0][0]*d[1][3][1][2] - v[0][1]*d[1][3][0][2] + v[0][2]*d[1][3][0][1];

		result[3][0] = v[0][1]*d[1][2][2][3] - v[0][2]*d[1][2][1][3] + v[0][3]*d[1][2][1][2];
		result[3][1] = v[0][0]*d[1][2][2][3] - v[0][2]*d[1][2][0][3] + v[0][3]*d[1][2][0][2];
		result[3][2] = v[0][0]*d[1][2][1][3] - v[0][1]*d[1][2][0][3] + v[0][3]*d[1][2][0][2];
		result[3][3] = v[0][0]*d[1][2][1][2] - v[0][1]*d[1][2][0][2] + v[0][2]*d[1][2][0][1];
		
		*this = result;
		for(int i=0;i<4;i++)
		{
			for(int j=0;j<4;j++)
			{
				if((i + j) %2 == 1)
					v[i][j] *= -1;
				v[i][j] *= inverseDet;
			}
		}
		Transpose();
	}

	void Matrix44f::Transpose()
	{
		swap(v[0][1],v[1][0]);
		swap(v[0][2],v[2][0]);
		swap(v[1][2],v[2][1]);
		swap(v[0][3],v[3][0]);
		swap(v[1][3],v[3][1]);
		swap(v[2][3],v[3][2]);
	}

	Matrix44f Matrix44f::GetInverse() const
	{
		Matrix44f x(*this);
		x.Invert();
		return x;
	}

	Matrix44f Matrix44f::GetTranspose() const
	{
		Matrix44f x(*this);
		x.Transpose();
		return x;
	}
	
Matrix44d::Matrix44d()
	{
	}

	Matrix44d::Matrix44d(double m[4][4])
	{
		v[0][0]=m[0][0]; v[0][1]=m[0][1]; v[0][2]=m[0][2]; v[0][3]=m[0][3];
		v[1][0]=m[1][0]; v[1][1]=m[1][1]; v[1][2]=m[1][2]; v[1][3]=m[1][3];
		v[2][0]=m[2][0]; v[2][1]=m[2][1]; v[2][2]=m[2][2]; v[2][3]=m[2][3];
		v[3][0]=m[3][0]; v[3][1]=m[3][1]; v[3][2]=m[3][2]; v[3][3]=m[3][3];
	}

	Matrix44d::Matrix44d(double m[16])
	{
		v[0][0]=m[0]; v[0][1]=m[1]; v[0][2]=m[2]; v[0][3]=m[3];
		v[1][0]=m[4]; v[1][1]=m[5]; v[1][2]=m[6]; v[1][3]=m[7];
		v[2][0]=m[8]; v[2][1]=m[9]; v[2][2]=m[10]; v[2][3]=m[11];
		v[3][0]=m[12]; v[3][1]=m[13]; v[3][2]=m[14]; v[3][3]=m[15];
	}

	Matrix44d::Matrix44d(Vector4d vec[4])
	{
		v[0] = vec[0];
		v[1] = vec[1];
		v[2] = vec[2];
		v[3] = vec[3];
	}

	Matrix44d::Matrix44d(const Matrix44d& m)
	{
		v[0] = m[0];
		v[1] = m[1];
		v[2] = m[2];
		v[3] = m[3];
	}

	Matrix44d Matrix44d::operator*(const Matrix44d& c) const
	{
		return Matrix44d(*this) *= c;
	}

	Vector4d Matrix44d::operator*(const Vector4d& c) const
	{
		Vector4d result;

		for(int i=0;i<4;i++)
		{
			result[i] = v[i].Dot(c);
		}
		return result;

	}

	Matrix44d Matrix44d::operator*(double f) const
	{
		Matrix44d result(*this);

		result[0]*=f;
		result[1]*=f;
		result[2]*=f;
		result[3]*=f;

		return result;
	}

	Matrix44d& Matrix44d::operator*=(const Matrix44d& c)
	{
		Matrix44d temp(*this);
		for(int i=0;i<4;i++)
		{		
			for(int j=0;j<4;j++)
			{
				double t=0;
				for(int k=0;k<4;k++)
					t += temp[i][k] * c[k][j];
				v[i][j] = t;
			}
		}
		return *this;
	}

	bool Matrix44d::operator==(const Matrix44d& m) const
	{
		return v[0][0]==m[0][0]&& v[0][1]==m[0][1]&& v[0][2]==m[0][2]&& v[0][3]==m[0][3]&&
		v[1][0]==m[1][0]&& v[1][1]==m[1][1]&& v[1][2]==m[1][2]&& v[1][3]==m[1][3]&&
		v[2][0]==m[2][0]&& v[2][1]==m[2][1]&& v[2][2]==m[2][2]&& v[2][3]==m[2][3]&&
		v[3][0]==m[3][0]&& v[3][1]==m[3][1]&& v[3][2]==m[3][2]&& v[3][3]==m[3][3];

	}

	void Matrix44d::MakeIdentity()
	{
		v[0][0]=1; v[0][1]=0; v[0][2]=0; v[0][3]=0;
		v[1][0]=0; v[1][1]=1; v[1][2]=0; v[1][3]=0;
		v[2][0]=0; v[2][1]=0; v[2][2]=1; v[2][3]=0;
		v[3][0]=0; v[3][1]=0; v[3][2]=0; v[3][3]=1;
	}

	void Matrix44d::MakeTranslate(const Vector3d& c)
	{
		v[0][0]=1; v[0][1]=0; v[0][2]=0; v[0][3]=c[0];
		v[1][0]=0; v[1][1]=1; v[1][2]=0; v[1][3]=c[1];
		v[2][0]=0; v[2][1]=0; v[2][2]=1; v[2][3]=c[2];
		v[3][0]=0; v[3][1]=0; v[3][2]=0; v[3][3]=1;
	}

	void Matrix44d::MakeTranslate(double x,double y,double z)
	{
		v[0][0]=1; v[0][1]=0; v[0][2]=0; v[0][3]=x;
		v[1][0]=0; v[1][1]=1; v[1][2]=0; v[1][3]=y;
		v[2][0]=0; v[2][1]=0; v[2][2]=1; v[2][3]=z;
		v[3][0]=0; v[3][1]=0; v[3][2]=0; v[3][3]=1;
	}

	void Matrix44d::MakeRotate(double radians,const Vector3d& axis_)
	{
		Vector3d axis = axis_;
		axis.Normalize();

		double c=cos(radians);
		double s=sin(radians);
		double t=1-c;
		v[0][0]=t*axis[0]*axis[0]+c;
		v[0][1]=t*axis[0]*axis[1]-s*axis[2];
		v[0][2]=t*axis[2]*axis[0]+s*axis[1];
		v[0][3]=0;
		v[1][0]=t*axis[0]*axis[1]+s*axis[2];
		v[1][1]=t*axis[1]*axis[1]+c;
		v[1][2]=t*axis[1]*axis[2]-s*axis[0];
		v[1][3]=0;
		v[2][0]=t*axis[2]*axis[0]-s*axis[1];
		v[2][1]=t*axis[1]*axis[2]+s*axis[0];
		v[2][2]=t*axis[2]*axis[2]+c;
		v[2][3]=0;
		v[3][0]=0;
		v[3][1]=0;
		v[3][2]=0;
		v[3][3]=1;
	}
	void Matrix44d::MakeRotate(double radians,double x,double y,double z)
	{	
		MakeRotate(radians,Vector3d(x,y,z));
	}
	
	void Matrix44d::MakeScale(const Vector3d& c)
	{
		v[0][0]=c[0];	v[0][1]=0;		v[0][2]=0;		v[0][3]=0;
		v[1][0]=0;		v[1][1]=c[1];	v[1][2]=0;		v[1][3]=0;
		v[2][0]=0;		v[2][1]=0;		v[2][2]=c[2];	v[2][3]=0;
		v[3][0]=0;		v[3][1]=0;		v[3][2]=0;		v[3][3]=1;
	}

	void Matrix44d::MakeScale(double x,double y,double z)
	{
		v[0][0]=x;	v[0][1]=0;	v[0][2]=0;	v[0][3]=0;
		v[1][0]=0;	v[1][1]=y;	v[1][2]=0;	v[1][3]=0;
		v[2][0]=0;	v[2][1]=0;	v[2][2]=z;	v[2][3]=0;
		v[3][0]=0;	v[3][1]=0;	v[3][2]=0;	v[3][3]=1;
	}

	double Matrix44d::Det4() const
	{
		return v[0][0]*Det3(1,2,3,1,2,3) - v[0][1]*Det3(1,2,3,0,2,3) + v[0][2]*Det3(1,2,3,0,1,3) - v[0][3]*Det3(1,2,3,0,1,2);
	}
	
	double Matrix44d::Det3(int i1,int i2,int i3,int j1,int j2,int j3) const
	{
		return v[i1][j1]*Det2(i2,i3,j2,j3)-v[i1][j2]*Det2(i2,i3,j1,j3)+v[i1][j3]*Det2(i2,i3,j1,j2);
	}

	double Matrix44d::Det2(int i1,int i2,int j1,int j2) const
	{
		return v[i1][j1]*v[i2][j2]-v[i1][j2]*v[i2][j1];
	}

	void Matrix44d::Invert()
	{
		double det = Det4();
		if(det == 0)
		{
			std::cerr<<"determinant is zero!!"<<std::endl;
			return;
		}
		double inverseDet = 1.0f/det;
		double d[4][4][4][4];
		for(int i1=0;i1<4;i1++)
			for(int i2=0;i2<4;i2++)
				for(int j1=0;j1<4;j1++)
					for(int j2=0;j2<4;j2++)
						d[i1][i2][j1][j2] = Det2(i1,i2,j1,j2);
		Matrix44d result;
		result[0][0] = v[1][1]*d[2][3][2][3] - v[1][2]*d[2][3][1][3] + v[1][3]*d[2][3][1][2];
		result[0][1] = v[1][0]*d[2][3][2][3] - v[1][2]*d[2][3][0][3] + v[1][3]*d[2][3][0][2];
		result[0][2] = v[1][0]*d[2][3][1][3] - v[1][1]*d[2][3][0][3] + v[1][3]*d[2][3][0][1];
		result[0][3] = v[1][0]*d[2][3][1][2] - v[1][1]*d[2][3][0][2] + v[1][2]*d[2][3][0][1];
		
		result[1][0] = v[0][1]*d[2][3][2][3] - v[0][2]*d[2][3][1][3] + v[0][3]*d[2][3][1][2];
		result[1][1] = v[0][0]*d[2][3][2][3] - v[0][2]*d[2][3][0][3] + v[0][3]*d[2][3][0][2];
		result[1][2] = v[0][0]*d[2][3][1][3] - v[0][1]*d[2][3][0][3] + v[0][3]*d[2][3][0][1];
		result[1][3] = v[0][0]*d[2][3][1][2] - v[0][1]*d[2][3][0][2] + v[0][2]*d[2][3][0][1];
		
		result[2][0] = v[0][1]*d[1][3][2][3] - v[0][2]*d[1][3][1][3] + v[0][3]*d[1][3][1][2];
		result[2][1] = v[0][0]*d[1][3][2][3] - v[0][2]*d[1][3][0][3] + v[0][3]*d[1][3][0][2];
		result[2][2] = v[0][0]*d[1][3][1][3] - v[0][1]*d[1][3][0][3] + v[0][3]*d[1][3][0][1];
		result[2][3] = v[0][0]*d[1][3][1][2] - v[0][1]*d[1][3][0][2] + v[0][2]*d[1][3][0][1];

		result[3][0] = v[0][1]*d[1][2][2][3] - v[0][2]*d[1][2][1][3] + v[0][3]*d[1][2][1][2];
		result[3][1] = v[0][0]*d[1][2][2][3] - v[0][2]*d[1][2][0][3] + v[0][3]*d[1][2][0][2];
		result[3][2] = v[0][0]*d[1][2][1][3] - v[0][1]*d[1][2][0][3] + v[0][3]*d[1][2][0][2];
		result[3][3] = v[0][0]*d[1][2][1][2] - v[0][1]*d[1][2][0][2] + v[0][2]*d[1][2][0][1];
		
		*this = result;
		for(int i=0;i<4;i++)
		{
			for(int j=0;j<4;j++)
			{
				if((i + j) %2 == 1)
					v[i][j] *= -1;
				v[i][j] *= inverseDet;
			}
		}
		Transpose();
	}

	void Matrix44d::Transpose()
	{
		swap(v[0][1],v[1][0]);
		swap(v[0][2],v[2][0]);
		swap(v[1][2],v[2][1]);
		swap(v[0][3],v[3][0]);
		swap(v[1][3],v[3][1]);
		swap(v[2][3],v[3][2]);
	}

	Matrix44d Matrix44d::GetInverse() const
	{
		Matrix44d x(*this);
		x.Invert();
		return x;
	}

	Matrix44d Matrix44d::GetTranspose() const
	{
		Matrix44d x(*this);
		x.Transpose();
		return x;
	}
	
} //namespace