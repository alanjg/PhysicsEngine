#ifndef SWIFTPOLYMESHBODY_H
#define SWIFTPOLYMESHBODY_H

#pragma comment(lib, "swiftpp.lib")

#include <vector>
#include <iostream>
#include "RigidBody.h"
#include "Swift.h"

namespace physics
{
	class SwiftPolyMeshBody: public RigidBody
	{
	public:
		SwiftPolyMeshBody(const Vector3& position, const Rotation& rot, bool mobile );

		virtual ~SwiftPolyMeshBody();
        virtual SwiftPolyMeshBody* Clone();

        virtual void Render();

        virtual void CopyTo(RigidBody* target)
        {
            SwiftPolyMeshBody* polyTarget = dynamic_cast<SwiftPolyMeshBody*>(target);
            RigidBody::CopyTo(target);
        }

		virtual void Print(std::ostream& out) const
		{
			out<<"<SwiftPolyMeshBody> pos:"<<Position()<<" vel:"<<velocity_;
		}

	private:
		static SWIFT_Scene swiftScene;
	};
}


#endif