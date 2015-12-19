//-----------------------------------------------------------------------------
// geom.h
//
// Defines the triangle and bbox structures for mesh preprocessing. These are
// unrelated to the data structures for the physics program itself.
//-----------------------------------------------------------------------------

#ifndef GEOM_H
#define GEOM_H

#include "Matrix33.h"
#include <limits>

namespace physics
{

	struct Tri
	{
		Vector3 p[3];
	};


	class BBox
	{
	public:
		// Keeping the transformations separate makes construction easier.
		// The rows of the rot matrix are also the basis vectors of the box.
		Matrix33 m_rot;
		Vector3  m_trans;

		// extents
		Vector3  m_ext; 

		unsigned int m_iLeft;  // indices into the tree vector
		unsigned int m_iRight;

		unsigned int m_iTri;   // index into the mesh's triangle array

		bool IsLeaf() const { return m_iLeft == std::numeric_limits<unsigned int>::max() &&
		                             m_iRight == std::numeric_limits<unsigned int>::max(); }
	};

}


#endif  // GEOM_H
