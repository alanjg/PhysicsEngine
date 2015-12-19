//-----------------------------------------------------------------------------
// jacobi.h
//
// This function is from Numerical Recipes in C++, second edition,
// section 11.1, page 468. It uses Jacobi transformations to find the
// eigenvectors of a symmetric matrix.
// The code below has been modified from the original. It has been adapted to
// our data structures, and optimized for 3x3 matrices.
//-----------------------------------------------------------------------------

#ifndef JACOBI_H
#define JACOBI_H

#include "Matrix33.h"
#include <cmath>

using namespace std;
using namespace physics;

inline void rot(Matrix33 &a, const double s, const double tau, const int i,
	const int j, const int k, const int l)
{
	double g, h;

	g = a[i][j];
	h = a[k][l];
	a[i][j] = g - s * (h + g * tau);
	a[k][l] = h + s * (g - h * tau);
}

// Returns true if successful.
// Fails if it doesn't have a solution after 50 iterations.
inline bool jacobi(Matrix33 &a, Vector3 &d, Matrix33 &v)
{
	double tresh,theta,tau,t,sm,s,h,g,c;

	Vector3 b( a[0][0], a[1][1], a[2][2] );
	Vector3 z( 0.0, 0.0, 0.0 );

	d = b;

	v.MakeIdentity();

	for( int i = 1; i <= 50; i++ )
	{
		sm = fabs(a[0][1]) + fabs(a[0][2]) + fabs(a[1][2]);
		if( sm == 0.0 )
			return true;

		if( i < 4 )
			tresh = 0.2 * sm / (3*3);
		else
			tresh = 0.0;

		// first iteration of unrolled loop. ip = 0, iq = 1
		g = 100.0 * fabs(a[0][1]);
        if( i > 4 && (fabs(d[0])+g) == fabs(d[0])
		          && (fabs(d[0])+g) == fabs(d[0]) )
		{
            a[0][1]=0.0;
		}
		else if( fabs(a[0][1]) > tresh )
		{
			h = d[1] - d[0];
			if( (fabs(h)+g) == fabs(h) )
			{
				t = (a[0][1]) / h;
			}
			else
			{
				theta = 0.5 * h / (a[0][1]);
				t = 1.0 / (fabs(theta) + sqrt(1.0 + theta*theta));
				if( theta < 0.0 ) t = -t;
			}
			c = 1.0 / sqrt(1 + t*t);
			s = t * c;
			tau = s / (1.0 + c);
			h = t * a[0][1];
			z[0] -= h;
			z[1] += h;
			d[0] -= h;
			d[1] += h;
			a[0][1]=0.0;
			rot(a,s,tau,0,2,1,2);
			rot(v,s,tau,0,0,0,1);
			rot(v,s,tau,1,0,1,1);
			rot(v,s,tau,2,0,2,1);
		}

		// second iteration of unrolled loop. ip = 0, iq = 2
		g = 100.0 * fabs(a[0][2]);
        if( i > 4 && (fabs(d[0])+g) == fabs(d[0])
		          && (fabs(d[0])+g) == fabs(d[0]) )
		{
            a[0][2]=0.0;
		}
		else if( fabs(a[0][2]) > tresh )
		{
			h = d[2] - d[0];
			if( (fabs(h)+g) == fabs(h) )
			{
				t = (a[0][2]) / h;
			}
			else
			{
				theta = 0.5 * h / (a[0][2]);
				t = 1.0 / (fabs(theta) + sqrt(1.0 + theta*theta));
				if( theta < 0.0 ) t = -t;
			}
			c = 1.0 / sqrt(1 + t*t);
			s = t * c;
			tau = s / (1.0 + c);
			h = t * a[0][2];
			z[0] -= h;
			z[2] += h;
			d[0] -= h;
			d[2] += h;
			a[0][2]=0.0;
			rot(a,s,tau,0,1,1,2);
			rot(v,s,tau,0,0,0,2);
			rot(v,s,tau,1,0,1,2);
			rot(v,s,tau,2,0,2,2);
		}

		// third iteration of unrolled loop. ip = 1, iq = 2
		g = 100.0 * fabs(a[1][2]);
        if( i > 4 && (fabs(d[1])+g) == fabs(d[1])
		          && (fabs(d[1])+g) == fabs(d[1]) )
		{
            a[1][2]=0.0;
		}
		else if( fabs(a[1][2]) > tresh )
		{
			h = d[2] - d[1];
			if( (fabs(h)+g) == fabs(h) )
			{
				t = (a[1][2]) / h;
			}
			else
			{
				theta = 0.5 * h / (a[1][2]);
				t = 1.0 / (fabs(theta) + sqrt(1.0 + theta*theta));
				if( theta < 0.0 ) t = -t;
			}
			c = 1.0 / sqrt(1 + t*t);
			s = t * c;
			tau = s / (1.0 + c);
			h = t * a[1][2];
			z[1] -= h;
			z[2] += h;
			d[1] -= h;
			d[2] += h;
			a[1][2]=0.0; 
			rot(a,s,tau,0,1,0,2);
			rot(v,s,tau,0,1,0,2);
			rot(v,s,tau,1,1,1,2);
			rot(v,s,tau,2,1,2,2);
		}

		b += z;
		d = b;
		z.Set(0.0, 0.0, 0.0);
	}

	return false;
}

void eigsrt(Vector3 &d, Matrix33 &v)
{
	int k;
	double p;

	// Just need to know which axis to split on, other two don't matter
//	for( int i = 0; i < 2; i++ )
	const int i = 0;
	{
		p = d[ k = i ];
		for( int j = i+1; j < 3; j++ )
			if( d[j] > p )
				p = d[ k = j ];
		if( k != i )
		{
			d[k] = d[i];
			d[i] = p;
			for( int j = 0; j < 3; j++ )
			{
				p = v[j][i];
				v[j][i] = v[j][k];
				v[j][k] = p;
			}
		}
	}
}

#endif  // JACOBI_H
