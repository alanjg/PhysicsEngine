#ifndef POLYMESHBODY_H
#define POLYMESHBODY_H

#include <vector>
#include <iostream>
#include "RigidBody.h"
#include "geom.h"

namespace physics
{
	class SphereBody;

	class PolyMeshBody: public RigidBody
	{
	public:
		PolyMeshBody( std::istream& input, const Vector3& position, const Rotation& rot, bool mobile );

		virtual ~PolyMeshBody();
        virtual PolyMeshBody* Clone();

        virtual void Render();

        virtual void CopyTo(RigidBody* target)
        {
            PolyMeshBody* polyTarget = dynamic_cast<PolyMeshBody*>(target);
            RigidBody::CopyTo(target);
			polyTarget->m_tris = m_tris;
			polyTarget->m_boxes = m_boxes;
			polyTarget->m_numTris = m_numTris;
			polyTarget->m_displayList = m_displayList;
			polyTarget->bi = bi;
        }

		virtual void Print(std::ostream& out) const
		{
			out<<"<PolyMeshBody> pos:"<<Position()<<" vel:"<<velocity_;
		}

		const BBox& GetBox( unsigned int index )       const  { return m_boxes[index]; }
		const Tri&  GetBoxTri( unsigned int boxIndex ) const  { return m_tris[m_boxes[boxIndex].m_iTri]; }

		std::vector<int> bi;

	private:
		std::vector<Tri>  m_tris;
		std::vector<BBox> m_boxes;
		unsigned int      m_numTris;

		unsigned int      m_displayList;

	};
}


#endif