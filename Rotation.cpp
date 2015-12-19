#include "Rotation.h"
#include "mathdefs.h"
#include <iostream>
#include <cassert>

namespace physics 
{

Rotationf::Rotationf()
{
}

Rotationf::Rotationf(const Vector3f& axis,float angle)
{
	axis_ = axis;
    axis_.Normalize();

    axis_ *= sinf(angle / 2.0f);
    angle_ = cosf(angle / 2.0f);
}

Rotationf::Rotationf(float q0,float q1,float q2,float q3):
	axis_(q0,q1,q2),angle_(q3)
{
	Normalize();
}

void Rotationf::Set(const Vector3f& axis,float angle)
{
	axis_ = axis;
    axis_.Normalize();

    axis_ *= sinf(angle / 2.0f);
    angle_ = cosf(angle / 2.0f);
}

void Rotationf::Set(float q0,float q1,float q2,float q3)
{
	axis_.Set(q0,q1,q2);
	angle_ = q3;
	Normalize();
}

void Rotationf::Set(const Vector3f& vfrom,const Vector3f& vto)
{
	Vector3f axis;
    float cost;

	Vector3f from = vfrom;
	Vector3f to = vto;

    from.Normalize();
    to.Normalize();
    cost = from.Dot(to);

    // check for degeneracies
    if (cost > 0.99999) // vectors are parallel
	{
		axis_.Set(0.0,0.0,0.0);
		angle_ = 1.0;
		return;
	}
	else if (cost < -0.99999) // vectors are opposite
	{
	// find an axis to rotate around, which should be
	// perpendicular to the original axis
	// Try cross product with (1,0,0) first, if that's one of our
	// original vectors then try  (0,1,0).
		Vector3f tmp = from.Cross(Vector3f(1.0, 0.0, 0.0));
		if (tmp.Magnitude() < 0.00001)
			tmp = from.Cross(Vector3f(0.0, 1.0, 0.0));

		tmp.Normalize();
		Set(tmp[0], tmp[1], tmp[2], 0.0);
		return ;
    }

    axis = vfrom.Cross(vto);
    axis.Normalize();

    // use half-angle formulae
    // sin^2 t = ( 1 - cos (2t) ) / 2
    axis *= sqrtf(0.5f * (1.0f - cost));

    // scale the axis by the sine of half the rotation angle to get
    // the normalized quaternion
    axis_ = axis;

    // cos^2 t = ( 1 + cos (2t) ) / 2
    // w part is cosine of half the rotation angle
    angle_ = sqrtf(0.5f * (1.0f + cost));
}

void Rotationf::Get(Vector3f& axis,float& angle) const
{
	float len=axis_.Magnitude();
	if(len > FLOAT_TOLERANCE)
	{
		axis = axis_ / len;
		angle = acosf(angle_)*2;		
	}
	else
	{
		axis.Set(0,0,1);
		angle=0.0;
	}
}

Vector3f Rotationf::GetAxis() const
{
	Vector3f axis;
	float angle;
	Get(axis,angle);
	return axis;
}

float Rotationf::GetAngle() const
{
	Vector3f axis;
	float angle;
	Get(axis,angle);
	return angle;
}

Rotationf Rotationf::Slerp(const Rotationf& r0,const Rotationf& r1,float t)
{
	Rotation rot;
	float omega, cosom, sinom;
	float s0, s1;
	Vector3f rot1ax;
	float rot1ang;
	// Calculate the cosine
	cosom = r0.axis_.Dot(r1.axis_) + r0.angle_*r1.angle_;

	// adjust signs if necessary
	if ( cosom < 0.0f ) 
	{
		cosom = -cosom;
		rot1ax = -r1.axis_;
		rot1ang = -r1.angle_;
	} 
	else  
	{
		rot1ax = r1.axis_;
		rot1ang = r1.angle_;
	}

	// calculate interpolating coeffs
	if ( (1.0f - cosom) > 0.00001f ) 
	{
		// standard case
		omega = acosf(cosom);
		sinom = sinf(omega);
		s0 = sinf((1.0f - t) * omega) / sinom;
		s1 = sinf(t * omega) / sinom;
	} 
	else 
	{        
		// rot0 and rot1 very close - just do linear interp.
		s0 = 1.0f - t;
		s1 = t;
	}

	// build the new quarternion
	rot.axis_ = r0.axis_*s0 + rot1ax*s1;
	rot.angle_= s0*r0.angle_ + s1*rot1ang;
	
	return rot;
}

Rotationf& Rotationf::Invert()
{
    float invNorm = 1.0f / Norm();

	assert( IsEqual(invNorm, 1.0f) );

	axis_ *= -invNorm;
    angle_ *= invNorm;
	return *this;
}

Rotationf Rotationf::Inverse() const
{
	Rotationf temp(*this);
	temp.Invert();
	return temp;
}

void Rotationf::ScaleAngle(float scale)
{
	angle_ *= scale;
	Normalize();
}


Rotationf& Rotationf::operator *=(const Rotationf& rhs)
{
	Vector3f crossProduct = axis_.Cross(rhs.axis_);
	crossProduct += rhs.axis_ * angle_ + axis_ * rhs.angle_;
	
	angle_ = angle_*rhs.angle_ - axis_.Dot(rhs.axis_);
	axis_ = crossProduct;
	return *this;
}


Rotationf operator*(const Rotationf& lhs,const Rotationf& rhs)
{
	return Rotationf(lhs) *= rhs;
}

float Rotationf::Norm()
{
	return axis_.Dot(axis_) + angle_*angle_;
}

void Rotationf::Normalize()
{
	float mag = Norm();
	if(IsZero(mag,FLOAT_TOLERANCE))
		return;
	mag = 1.0f / sqrtf(mag);
	axis_ /= mag;
	angle_ /= mag;
}

Vector3f Rotationf::Rotate(const Vector3f& vec) const
{
	Rotationf p;
	p.axis_ = vec;
	p.angle_ = 0;
	Rotationf inv = Inverse();
	Rotationf result = *this * p;
	result *= inv;
	return result.axis_;
}

Vector3f Rotationf::InverseRotate(const Vector3f& vec) const
{
	Rotationf p;
	p.axis_ = vec;
	p.angle_ = 0;
	Rotationf inv = Inverse();
	Rotationf result = inv * p;
	result *= *this;
	return result.axis_;
}

std::ostream& operator<<(std::ostream& out,const Rotationf& rot)
{
	Vector3f axis;
	float angle;
	rot.Get(axis,angle);
	return out << axis << "," << angle;
}


Rotationd::Rotationd()
{
}

Rotationd::Rotationd(const Vector3d& axis,double angle)
{
	axis_ = axis;
    axis_.Normalize();

    axis_ *= sin(angle / 2.0);
    angle_ = cos(angle / 2.0);
}

Rotationd::Rotationd(double q0,double q1,double q2,double q3):
	axis_(q0,q1,q2),angle_(q3)
{
	Normalize();
}

void Rotationd::Set(const Vector3d& axis,double angle)
{
	axis_ = axis;
    axis_.Normalize();

    axis_ *= sin(angle / 2.0);
    angle_ = cos(angle / 2.0);
}

void Rotationd::Set(double q0,double q1,double q2,double q3)
{
	axis_.Set(q0,q1,q2);
	angle_ = q3;
	Normalize();
}

void Rotationd::Set(const Vector3d& vfrom,const Vector3d& vto)
{
	Vector3d axis;
    double cost;

	Vector3d from = vfrom;
	Vector3d to = vto;

    from.Normalize();
    to.Normalize();
    cost = from.Dot(to);

    // check for degeneracies
    if (cost > 0.99999) // vectors are parallel
	{
		axis_.Set(0.0,0.0,0.0);
		angle_ = 1.0;
		return;
	}
	else if (cost < -0.99999) // vectors are opposite
	{
	// find an axis to rotate around, which should be
	// perpendicular to the original axis
	// Try cross product with (1,0,0) first, if that's one of our
	// original vectors then try  (0,1,0).
		Vector3d tmp = from.Cross(Vector3d(1.0, 0.0, 0.0));
		if (tmp.Magnitude() < 0.00001)
			tmp = from.Cross(Vector3d(0.0, 1.0, 0.0));

		tmp.Normalize();
		Set(tmp[0], tmp[1], tmp[2], 0.0);
		return ;
    }

    axis = vfrom.Cross(vto);
    axis.Normalize();

    // use half-angle formulae
    // sin^2 t = ( 1 - cos (2t) ) / 2
    axis *= sqrt(0.5 * (1.0 - cost));

    // scale the axis by the sine of half the rotation angle to get
    // the normalized quaternion
    axis_ = axis;

    // cos^2 t = ( 1 + cos (2t) ) / 2
    // w part is cosine of half the rotation angle
    angle_ = sqrt(0.5 * (1.0 + cost));
}

void Rotationd::Get(Vector3d& axis,double& angle) const
{
	double len=axis_.Magnitude();
	if(len > DOUBLE_TOLERANCE)
	{
		axis = axis_ / len;
		angle = acos(angle_)*2;		
	}
	else
	{
		axis.Set(0,0,1);
		angle=0.0;
	}
}

Vector3d Rotationd::GetAxis() const
{
	Vector3d axis;
	double angle;
	Get(axis,angle);
	return axis;
}

double Rotationd::GetAngle() const
{
	Vector3d axis;
	double angle;
	Get(axis,angle);
	return angle;
}

Rotationd Rotationd::Slerp(const Rotationd& r0,const Rotationd& r1,double t)
{
	Rotationd rot;
	double omega, cosom, sinom;
	double s0, s1;
	Vector3d rot1ax;
	double rot1ang;
	// Calculate the cosine
	cosom = r0.axis_.Dot(r1.axis_) + r0.angle_*r1.angle_;

	// adjust signs if necessary
	if ( cosom < 0.0f ) 
	{
		cosom = -cosom;
		rot1ax = -r1.axis_;
		rot1ang = -r1.angle_;
	} 
	else  
	{
		rot1ax = r1.axis_;
		rot1ang = r1.angle_;
	}

	// calculate interpolating coeffs
	if ( (1.0f - cosom) > 0.00001f ) 
	{
		// standard case
		omega = acos(cosom);
		sinom = sin(omega);
		s0 = sin((1.0f - t) * omega) / sinom;
		s1 = sin(t * omega) / sinom;
	} 
	else 
	{        
		// rot0 and rot1 very close - just do linear interp.
		s0 = 1.0f - t;
		s1 = t;
	}

	// build the new quarternion
	rot.axis_ = r0.axis_*s0 + rot1ax*s1;
	rot.angle_= s0*r0.angle_ + s1*rot1ang;
	
	return rot;
}

Rotationd& Rotationd::Invert()
{
    double invNorm = 1.0 / Norm();
	
	assert( IsEqual(invNorm, 1.0) );

	axis_ *= -invNorm;
    angle_ *= invNorm;
	return *this;
}

Rotationd Rotationd::Inverse() const
{
	Rotationd temp(*this);
	temp.Invert();
	return temp;
}

void Rotationd::ScaleAngle(double scale)
{
	angle_ *= scale;
	Normalize();
}


Rotationd& Rotationd::operator *=(const Rotationd& rhs)
{
	Vector3d crossProduct = axis_.Cross(rhs.axis_);
	crossProduct += rhs.axis_ * angle_ + axis_ * rhs.angle_;
	
	angle_ = angle_*rhs.angle_ - axis_.Dot(rhs.axis_);
	axis_ = crossProduct;
	return *this;
}


Rotationd operator*(const Rotationd& lhs,const Rotationd& rhs)
{
	return Rotationd(lhs) *= rhs;
}

double Rotationd::Norm()
{
	return axis_.Dot(axis_) + angle_*angle_;
}

void Rotationd::Normalize()
{
	double mag = Norm();
	if(IsZero(mag,DOUBLE_TOLERANCE))
		return;
	mag = 1.0f / sqrt(mag);
	axis_ /= mag;
	angle_ /= mag;
}

Vector3d Rotationd::Rotate(const Vector3d& vec) const
{
	Rotationd p;
	p.axis_ = vec;
	p.angle_ = 0;
	Rotationd inv = Inverse();
	Rotationd result = *this * p;
	result *= inv;
	return result.axis_;
}

Vector3d Rotationd::InverseRotate(const Vector3d& vec) const
{
	Rotationd p;
	p.axis_ = vec;
	p.angle_ = 0;
	Rotationd inv = Inverse();
	Rotationd result = inv * p;
	result *= *this;
	return result.axis_;
}

std::ostream& operator<<(std::ostream& out,const Rotationd& rot)
{
	Vector3d axis;
	double angle;
	rot.Get(axis,angle);
	return out << axis << "," << angle;
}
}