#ifndef MATRIX44_H
#define MATRIX44_H

#include "Vector4.h"
namespace physics
{

class Vector3f;
class Matrix44f
{
public:
	Matrix44f();
	Matrix44f(float m[4][4]);
	Matrix44f(float m[16]);
	Matrix44f(Vector4f vec[4]); 
	Matrix44f(const Matrix44f& m);

	float& operator()(int i,int j) { return v[i][j]; }
	const float& operator()(int i,int j) const { return v[i][j]; }
	Vector4f& operator[](int index) { return v[index]; }
	const Vector4f& operator[](int index) const { return v[index]; }

	Matrix44f operator*(const Matrix44f&) const;
	Vector4f operator*(const Vector4f&) const;
	Matrix44f operator*(float f) const;

	Matrix44f& operator*=(const Matrix44f&);
	bool operator==(const Matrix44f&rhs) const;

	void MakeIdentity();
	void MakeTranslate(const Vector3f& translate);
	void MakeTranslate(float x,float y,float z);
	void MakeRotate(float radians,const Vector3f& axis);
	void MakeRotate(float radians,float x,float y,float z);
	void MakeScale(const Vector3f&);
	void MakeScale(float x,float y,float z);

	void Invert();
	void Transpose();
	Matrix44f GetInverse() const;
	Matrix44f GetTranspose() const;

private:
	
	float Det2(int i1,int i2,int j1,int j2) const;
	float Det3(int i1,int i2,int i3,int j1,int j2,int j3) const;
	float Det4() const;
	
	Vector4f v[4];
};

class Vector3d;
class Matrix44d
{
public:
	Matrix44d();
	Matrix44d(double m[4][4]);
	Matrix44d(double m[16]);
	Matrix44d(Vector4d vec[4]); 
	Matrix44d(const Matrix44d& m);

	double& operator()(int i,int j) { return v[i][j]; }
	const double& operator()(int i,int j) const { return v[i][j]; }
	Vector4d& operator[](int index) { return v[index]; }
	const Vector4d& operator[](int index) const { return v[index]; }

	Matrix44d operator*(const Matrix44d&) const;
	Vector4d operator*(const Vector4d&) const;
	Matrix44d operator*(double f) const;

	Matrix44d& operator*=(const Matrix44d&);
	bool operator==(const Matrix44d&rhs) const;

	void MakeIdentity();
	void MakeTranslate(const Vector3d& translate);
	void MakeTranslate(double x,double y,double z);
	void MakeRotate(double radians,const Vector3d& axis);
	void MakeRotate(double radians,double x,double y,double z);
	void MakeScale(const Vector3d&);
	void MakeScale(double x,double y,double z);

	void Invert();
	void Transpose();
	Matrix44d GetInverse() const;
	Matrix44d GetTranspose() const;

private:
	
	double Det2(int i1,int i2,int j1,int j2) const;
	double Det3(int i1,int i2,int i3,int j1,int j2,int j3) const;
	double Det4() const;
	
	Vector4d v[4];
};

typedef Matrix44f Matrix44;

} //namespace

#endif //file