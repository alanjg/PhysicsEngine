//-----------------------------------------------------------------------------
// BBoxTree.cpp
//
// Functions for building a bounding box tree of a mesh
//-----------------------------------------------------------------------------

#define DOUBLE_PRECISION

#include "Matrix33.h"
#include "geom.h"
#include "jacobi.h"
#include <string>
#include <vector>
#include <algorithm>
#include <numeric>
#include <limits>
#include <fstream>
#include <cassert>

using namespace std;
using namespace physics;


// *gasp* statics? uh oh! not thread safe!!
static vector<BBox>     boxes;
static int              iNextFreeBox;
static vector<int>      triIndices;
// each triangle's contribution to the (sub)mesh's mean and covariance matrix
static vector<Vector3>  triMeans;
static vector<Matrix33> covContrib;
static vector<double>   triAreas;

std::vector<Tri> tris;
unsigned int numtris;


void CalcTriStats()
{
	triMeans.resize(numtris);
	covContrib.resize(numtris);
	triAreas.resize(numtris);

	// find the contribution of each triangle to the mean of the mesh
	Vector3 edge1, edge2, normal;
	for( unsigned int i = 0; i < numtris; i++ )
	{
		// compute the area of the triangle
		Vector3 p0 = tris[i].p[0];
		Vector3 p1 = tris[i].p[1];
		Vector3 p2 = tris[i].p[2];
		edge1 = tris[i].p[1] - tris[i].p[0];
		edge2 = tris[i].p[2] - tris[i].p[0];
		normal = edge1.Cross(edge2);
		triAreas[i] = 0.5 * normal.Magnitude();

		triMeans[i] = ( tris[i].p[0] + tris[i].p[1] + tris[i].p[2] ) / 3;
		// We actually need to weight it by the triangle area also.
		// This is done below so we can use the unweighted versions
		// for the covariance computation.
	}

	// find the contribution of each triangle to the covariance matrix
	for( unsigned int i = 0; i < numtris; i++ )
	{
		// This is an unrolled 3x3 loop that iterates over the elements of the matrix
		// The formula is from Faloutsos's Dance code
		covContrib[i][0][0] = triAreas[i] *
		( 9 * triMeans[i][0] * triMeans[i][0] + tris[i].p[0][0] * tris[i].p[0][0]
		  + tris[i].p[1][0] * tris[i].p[1][0] + tris[i].p[2][0] * tris[i].p[2][0] ) / 12;

		covContrib[i][0][1] = triAreas[i] *
		( 9 * triMeans[i][0] * triMeans[i][1] + tris[i].p[0][0] * tris[i].p[0][1]
		  + tris[i].p[1][0] * tris[i].p[1][1] + tris[i].p[2][0] * tris[i].p[2][1] ) / 12;

		covContrib[i][0][2] = triAreas[i] *
		( 9 * triMeans[i][0] * triMeans[i][2] + tris[i].p[0][0] * tris[i].p[0][2]
		  + tris[i].p[1][0] * tris[i].p[1][2] + tris[i].p[2][0] * tris[i].p[2][2] ) / 12;

		covContrib[i][1][1] = triAreas[i] *
		( 9 * triMeans[i][1] * triMeans[i][1] + tris[i].p[0][1] * tris[i].p[0][1]
		  + tris[i].p[1][1] * tris[i].p[1][1] + tris[i].p[2][1] * tris[i].p[2][1] ) / 12;

		covContrib[i][1][2] = triAreas[i] *
		( 9 * triMeans[i][1] * triMeans[i][2] + tris[i].p[0][1] * tris[i].p[0][2]
		  + tris[i].p[1][1] * tris[i].p[1][2] + tris[i].p[2][1] * tris[i].p[2][2] ) / 12;

		covContrib[i][2][2] = triAreas[i] *
		( 9 * triMeans[i][2] * triMeans[i][2] + tris[i].p[0][2] * tris[i].p[0][2]
		  + tris[i].p[1][2] * tris[i].p[1][2] + tris[i].p[2][2] * tris[i].p[2][2] ) / 12;

		// Fill out the lower half (it's symmetric)
		covContrib[i][1][0] = covContrib[i][0][1];
		covContrib[i][2][0] = covContrib[i][0][2];
		covContrib[i][2][1] = covContrib[i][1][2];

		// For later
		triMeans[i] *= triAreas[i];
	}
}

inline void BoxIn( Vector3& boxMin, Vector3& boxMax, const Vector3& point )
{
	if( point[0] < boxMin[0] ) boxMin[0] = point[0];
	if( point[1] < boxMin[1] ) boxMin[1] = point[1];
	if( point[2] < boxMin[2] ) boxMin[2] = point[2];
	if( point[0] > boxMax[0] ) boxMax[0] = point[0];
	if( point[1] > boxMax[1] ) boxMax[1] = point[1];
	if( point[2] > boxMax[2] ) boxMax[2] = point[2];
}

// This function wraps up the SplitBox recursion
bool LeafBox( int iBox,   // index of the box
              int iTri )  // index of the triangle inside the box
{
	boxes[iBox].m_iLeft = numeric_limits<unsigned int>::max();
	boxes[iBox].m_iRight = numeric_limits<unsigned int>::max();

	boxes[iBox].m_iTri = triIndices[iTri];

	// Find the longest edge and make it the major axis of the box
	Vector3 edge1 = tris[ triIndices[iTri] ].p[1] - tris[ triIndices[iTri] ].p[0];
	Vector3 edge2 = tris[ triIndices[iTri] ].p[2] - tris[ triIndices[iTri] ].p[0];
	Vector3 edge3 = tris[ triIndices[iTri] ].p[2] - tris[ triIndices[iTri] ].p[1];
	double len1 = edge1.MagnitudeSquared();
	double len2 = edge2.MagnitudeSquared();
	double len3 = edge3.MagnitudeSquared();

	Vector3 majorAxis;
	if( len1 > len2 && len1 > len3 )
	{
		majorAxis = edge1 / sqrt(len1);  // normalized
	}
	else if( len2 > len1 && len2 > len3 )
	{
		majorAxis = edge2 / sqrt(len2);
	}
	else // ( len3 > len1 && len3 > len2 )
	{
		majorAxis = edge3 / sqrt(len3);
	}

	// The normal of the triangle is the last axis
	Vector3 normal = edge1.Cross(edge2);
	normal.Normalize();

	Vector3 secondAxis = majorAxis.Cross(normal);
	assert( IsEqual(secondAxis.MagnitudeSquared(), 1.0) );

	// Make the rotation matrix
	boxes[iBox].m_rot.Set( majorAxis, secondAxis, normal );

	// Get the extents
	Vector3 boxMin, boxMax;
	Vector3 mean;
	boxMin = boxMax = boxes[iBox].m_rot * tris[ triIndices[iTri] ].p[0];
	BoxIn( boxMin, boxMax, boxes[iBox].m_rot * tris[ triIndices[iTri] ].p[1] );
	BoxIn( boxMin, boxMax, boxes[iBox].m_rot * tris[ triIndices[iTri] ].p[2] );

	mean = 0.5 * (boxMin + boxMax);
	boxes[iBox].m_trans = mean * boxes[iBox].m_rot;
	boxes[iBox].m_ext = 0.5 * (boxMax - boxMin);

	return true;
}

bool SplitBox( int iBox,   // index of the parent box
               int iTris,  // starting index of the contained triangles
               int n )     // number of tris in the parent box
{
	if( n == 1 )
		return LeafBox( iBox, iTris );

	Vector3 boxMin, boxMax;
	Vector3 curMean;
	int nLeft = 0;  // number of tris in the left child

	double leftArea = 0.0;
	Vector3 leftMean(0.0, 0.0, 0.0);
	Matrix33 leftCov(0.0, 0.0, 0.0,
	                 0.0, 0.0, 0.0,
				     0.0, 0.0, 0.0);
	double rightArea = 0.0;
	Vector3 rightMean(0.0, 0.0, 0.0);
	Matrix33 rightCov(0.0, 0.0, 0.0,
	                  0.0, 0.0, 0.0,
				      0.0, 0.0, 0.0);

	// The splitting axis is the first row of the parent
	// box's rotation matrix. We will project everything
	// onto this axis and do all the comparisons in 1d.
	Vector3 splitAxis( boxes[iBox].m_rot[0] );
	double meanXval = boxes[iBox].m_trans.Dot( splitAxis );  // x val in box space

	boxMin = boxMax = boxes[iBox].m_rot * tris[ triIndices[iTris] ].p[0];
	if( n != 2 )
	{
		for( int i = 0; i < n; i++ )
		{
			// Boxing in all the points (in box space, of course)
			BoxIn( boxMin, boxMax, boxes[iBox].m_rot * tris[ triIndices[iTris+i] ].p[0] );
			BoxIn( boxMin, boxMax, boxes[iBox].m_rot * tris[ triIndices[iTris+i] ].p[1] );
			BoxIn( boxMin, boxMax, boxes[iBox].m_rot * tris[ triIndices[iTris+i] ].p[2] );

			// should probably save this somewhere
			// Note: triMeans[ triIndices[iTris+i] ] holds (mean * area)
			curMean = ( tris[ triIndices[iTris+i] ].p[0] +
						tris[ triIndices[iTris+i] ].p[1] +
						tris[ triIndices[iTris+i] ].p[2] ) / 3;

			if( curMean.Dot(splitAxis) < meanXval )
			{
				// Put it in the left group
				leftMean += triMeans[ triIndices[iTris+i] ];
				leftCov += covContrib[ triIndices[iTris+i] ];
				leftArea += triAreas[ triIndices[iTris+i] ];

				std::swap( triIndices[iTris+i], triIndices[iTris+nLeft] );
				nLeft++;

			}
			else
			{
				// Put it in the right group
				rightMean += triMeans[ triIndices[iTris+i] ];
				rightCov += covContrib[ triIndices[iTris+i] ];
				rightArea += triAreas[ triIndices[iTris+i] ];
			}
		}
	}
	else
	{
		// First triangle
		//BoxIn( boxMin, boxMax, boxes[iBox].m_rot * tris[ triIndices[iTris] ].p[0] );
		BoxIn( boxMin, boxMax, boxes[iBox].m_rot * tris[ triIndices[iTris] ].p[1] );
		BoxIn( boxMin, boxMax, boxes[iBox].m_rot * tris[ triIndices[iTris] ].p[2] );

		// Put it in the left group
		leftMean += triMeans[ triIndices[iTris] ];
		leftCov += covContrib[ triIndices[iTris] ];
		leftArea += triAreas[ triIndices[iTris] ];
		nLeft++;

		// Second triangle
		BoxIn( boxMin, boxMax, boxes[iBox].m_rot * tris[ triIndices[iTris+1] ].p[0] );
		BoxIn( boxMin, boxMax, boxes[iBox].m_rot * tris[ triIndices[iTris+1] ].p[1] );
		BoxIn( boxMin, boxMax, boxes[iBox].m_rot * tris[ triIndices[iTris+1] ].p[2] );

		// Put it in the right group
		rightMean += triMeans[ triIndices[iTris+1] ];
		rightCov += covContrib[ triIndices[iTris+1] ];
		rightArea += triAreas[ triIndices[iTris+1] ];
	}

	assert( nLeft != 0 && nLeft != n );

	leftCov -= ( Matrix33(leftMean[0]*leftMean[0],leftMean[0]*leftMean[1],leftMean[0]*leftMean[2],
	                      leftMean[1]*leftMean[0],leftMean[1]*leftMean[1],leftMean[1]*leftMean[2],
	                      leftMean[2]*leftMean[0],leftMean[2]*leftMean[1],leftMean[2]*leftMean[2]) ) / leftArea;
	leftMean /= leftArea;
	rightCov -= ( Matrix33(rightMean[0]*rightMean[0],rightMean[0]*rightMean[1],rightMean[0]*rightMean[2],
	                       rightMean[1]*rightMean[0],rightMean[1]*rightMean[1],rightMean[1]*rightMean[2],
	                       rightMean[2]*rightMean[0],rightMean[2]*rightMean[1],rightMean[2]*rightMean[2]) ) / rightArea;
	rightMean /= rightArea;

	// Now reset the trans of the parent box to the center of the box
	// and set the dimensions variable.
	boxes[iBox].m_trans = ( 0.5 * (boxMin + boxMax) ) * boxes[iBox].m_rot;
	boxes[iBox].m_ext = 0.5 * (boxMax - boxMin);

	boxes[iBox].m_iLeft = iNextFreeBox++;
	boxes[iBox].m_iRight = iNextFreeBox++;

	boxes[iBox].m_iTri = numeric_limits<unsigned int>::max();

	// Get the basis vectors for the child boxes
	Matrix33 eigVects;
	Vector3  eigVals;

	if( nLeft > 1 )
	{
		jacobi( leftCov, eigVals, eigVects );
		eigsrt( eigVals, eigVects );
		boxes[ boxes[iBox].m_iLeft ].m_trans = leftMean;
		boxes[ boxes[iBox].m_iLeft ].m_rot = eigVects.Transpose();

		if( !SplitBox(boxes[iBox].m_iLeft, iTris, nLeft) )
			return false;
	}
	else
	{
		if( !LeafBox(boxes[iBox].m_iLeft, iTris) )
			return false;
	}

	if( (n-nLeft) > 1 )
	{      
		jacobi( rightCov, eigVals, eigVects );
		eigsrt( eigVals, eigVects );
		boxes[ boxes[iBox].m_iRight ].m_trans = rightMean;
		boxes[ boxes[iBox].m_iRight ].m_rot = eigVects.Transpose();

		if( !SplitBox(boxes[iBox].m_iRight, iTris + nLeft, n - nLeft) )
			return false;
	}
	else
	{
		if( !LeafBox(boxes[iBox].m_iRight, iTris + nLeft) )
			return false;
	}

	return true;
}

// Returns true if successful
bool BuildBBoxTree()
{
	Vector3 mean(0.0, 0.0, 0.0);
	Matrix33 cov(0.0, 0.0, 0.0,  // the covariance matrix of the mesh
	             0.0, 0.0, 0.0,
				 0.0, 0.0, 0.0);

	boxes.resize(numtris * 2 - 1);
	triIndices.resize(numtris);

	iNextFreeBox = 1;

	for( unsigned int i = 0; i < numtris; i++ )
		triIndices[i] = i;

	// Get the mean and covariance of the root mesh
	CalcTriStats();
	double totalArea = accumulate( triAreas.begin(), triAreas.end(), 0.0 );
	mean = accumulate( triMeans.begin(), triMeans.end(), mean );
	cov = accumulate( covContrib.begin(), covContrib.end(), cov ) -
	      ( Matrix33(mean[0]*mean[0],mean[0]*mean[1],mean[0]*mean[2],
	                 mean[1]*mean[0],mean[1]*mean[1],mean[1]*mean[2],
	                 mean[2]*mean[0],mean[2]*mean[1],mean[2]*mean[2]) ) / totalArea;
	mean /= totalArea;

	// Compute the eigenvectors of the covariance matrix
	Matrix33 eigVects;
	Vector3  eigVals;
	jacobi( cov, eigVals, eigVects );

	// We need to know which axis is the largest, to split on it
	eigsrt( eigVals, eigVects );

	// The eigenvectors are in the columns of eigVects. We want rows.
	boxes[0].m_rot = eigVects.Transpose();
	boxes[0].m_trans = mean;

	// Recurse
	return SplitBox( 0, 0, numtris );
}

// Returns true if successful
bool WriteBBoxTree( ofstream& outStream )
{
	float f[9];
	outStream.write( (char*)&numtris, sizeof(unsigned int) );
	for( unsigned int i = 0; i < numtris; i++ )
	{
		f[0] = (float)tris[i].p[0][0];
		f[1] = (float)tris[i].p[0][1];
		f[2] = (float)tris[i].p[0][2];
		f[3] = (float)tris[i].p[1][0];
		f[4] = (float)tris[i].p[1][1];
		f[5] = (float)tris[i].p[1][2];
		f[6] = (float)tris[i].p[2][0];
		f[7] = (float)tris[i].p[2][1];
		f[8] = (float)tris[i].p[2][2];
		outStream.write( (char*)f, 9 * sizeof(float) );
	}

	unsigned int numBoxes = (unsigned int)boxes.size();
	outStream.write( (char*)&numBoxes, sizeof(unsigned int) );

	for( unsigned int i = 0; i < numBoxes; i++ )
	{
		f[0] = (float)boxes[i].m_rot[0][0];
		f[1] = (float)boxes[i].m_rot[0][1];
		f[2] = (float)boxes[i].m_rot[0][2];
		f[3] = (float)boxes[i].m_rot[1][0];
		f[4] = (float)boxes[i].m_rot[1][1];
		f[5] = (float)boxes[i].m_rot[1][2];
		f[6] = (float)boxes[i].m_rot[2][0];
		f[7] = (float)boxes[i].m_rot[2][1];
		f[8] = (float)boxes[i].m_rot[2][2];
		outStream.write( (char*)f, 9 * sizeof(float) );

		f[0] = (float)boxes[i].m_trans[0];
		f[1] = (float)boxes[i].m_trans[1];
		f[2] = (float)boxes[i].m_trans[2];
		f[3] = (float)boxes[i].m_ext[0];
		f[4] = (float)boxes[i].m_ext[1];
		f[5] = (float)boxes[i].m_ext[2];
		outStream.write( (char*)f, 6 * sizeof(float) );

		outStream.write( (char*)&boxes[i].m_iLeft, sizeof(unsigned int) );
		outStream.write( (char*)&boxes[i].m_iRight, sizeof(unsigned int) );
		outStream.write( (char*)&boxes[i].m_iTri, sizeof(unsigned int) );
	}

	return outStream.good();
}
