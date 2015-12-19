#include "SwiftPolyMeshBody.h"
#include "Constants.h"
#include "Vector2.h"
#include "Vector3.h"
#include <gl/glut.h>
#include <vector>
using namespace std;

namespace physics
{

SwiftPolyMeshBody::SwiftPolyMeshBody( const Vector3& position, const Rotation& rot, bool mobile ) :
	RigidBody( position, rot, mobile )
{
	
}

void SwiftPolyMeshBody::Render()
{
	

}

SwiftPolyMeshBody::~SwiftPolyMeshBody()
{
    
}

SwiftPolyMeshBody* SwiftPolyMeshBody::Clone()
{
    SwiftPolyMeshBody* p = new SwiftPolyMeshBody(*this);
    return p;
}

} // namespace physics
