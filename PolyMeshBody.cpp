#include "PolyMeshBody.h"
#include "Constants.h"
#include "Vector2.h"
#include "Vector3.h"
#include <gl/glut.h>
#include <vector>
using namespace std;

namespace physics
{

struct MeshDrawInfo
{
	unsigned int numIndices;
	unsigned int numVerts;

	unsigned int* indices;
	Vector2* texCoords;
	Vector3* normals;
	Vector3* vertices;

	MeshDrawInfo() : numIndices(0), numVerts(0), indices(0), texCoords(0), normals(0), vertices(0) {}
	~MeshDrawInfo()
	{
		delete [] indices;
		delete [] texCoords;
		delete [] normals;
		delete [] vertices;
	}
};

PolyMeshBody::PolyMeshBody( std::istream& input, const Vector3& position, const Rotation& rot, bool mobile ) :
	RigidBody( position, rot, mobile )
{
	unsigned int i;

	char header[4] = "xxx";
	input.read( header, 3 );
	if( strcmp(header, "amf") != 0 )
	{
		cerr << "Invalid mesh file\n" << endl;
		m_numTris = 0;
		return;
	}

	MeshDrawInfo drawInfo;

	input.read( (char*)&drawInfo.numIndices, sizeof(unsigned int) );
	input.read( (char*)&drawInfo.numVerts, sizeof(unsigned int) );

	drawInfo.indices   = new unsigned int[drawInfo.numIndices];
	drawInfo.texCoords = new Vector2[drawInfo.numVerts];
	drawInfo.normals   = new Vector3[drawInfo.numVerts];
	drawInfo.vertices  = new Vector3[drawInfo.numVerts];

	for( i = 0; i < drawInfo.numIndices; i++ )
		input.read( (char*)&drawInfo.indices[i], sizeof(unsigned int) );
	for( i = 0; i < drawInfo.numVerts; i++ )
		input.read( (char*)&drawInfo.vertices[i], 3 * sizeof(float) );
	for( i = 0; i < drawInfo.numVerts; i++ )
		input.read( (char*)&drawInfo.normals[i], 3 * sizeof(float) );
	if( input.peek() != 0xff )
	{
		for( i = 0; i < drawInfo.numVerts; i++ )
			input.read( (char*)&drawInfo.texCoords[i][0], 2 * sizeof(float) );
	}
	input.get();

	m_displayList = glGenLists(1);
	glNewList( m_displayList, GL_COMPILE );
		glVertexPointer( 3, GL_FLOAT, 0, drawInfo.vertices );
		glNormalPointer( GL_FLOAT, 0, drawInfo.normals );
		glDrawElements( GL_TRIANGLES, drawInfo.numIndices, GL_UNSIGNED_INT, drawInfo.indices );
	glEndList();

	input.read( (char*)&m_numTris, sizeof(unsigned int) );
	m_tris.resize( m_numTris );
	float p[9];
	for( i = 0; i < m_numTris; i++ )
	{
		input.read( (char*)p, 9 * sizeof(float) );
		m_tris[i].p[0].Set( p[0], p[1], p[2] );
		m_tris[i].p[1].Set( p[3], p[4], p[5] );
		m_tris[i].p[2].Set( p[6], p[7], p[8] );
	}

	unsigned int numBoxes;
	input.read( (char*)&numBoxes, sizeof(unsigned int) );
	m_boxes.resize( numBoxes );
	for( i = 0; i < numBoxes; i++ )
	{
		input.read( (char*)p, 9 * sizeof(float) );
		m_boxes[i].m_rot.Set( p );
		input.read( (char*)p, 6 * sizeof(float) );
		m_boxes[i].m_trans.Set( p );
		m_boxes[i].m_ext.Set( &p[3] );
		m_boxes[i].m_ext[0] += 0.5f * COLLISION_EPSILON;
		m_boxes[i].m_ext[1] += 0.5f * COLLISION_EPSILON;
		m_boxes[i].m_ext[2] += 0.5f * COLLISION_EPSILON;

		input.read( (char*)&m_boxes[i].m_iLeft, sizeof(unsigned int) );
		input.read( (char*)&m_boxes[i].m_iRight, sizeof(unsigned int) );
		input.read( (char*)&m_boxes[i].m_iTri, sizeof(unsigned int) );
	}

	if( mobile )
	{
		input.read( (char*)&inverseMass_, sizeof(float) );
		input.read( (char*)&restitution_, sizeof(float) );
		float f;
		input.read( (char*)&f, sizeof(float) );
		inertia_ *= f;
		inverseInertia_ /= f;
	}
}

void PolyMeshBody::Render()
{
	Vector3 normal;

	Vector3 axis;
	float angle;
	Vector3 pos = Position();
	Orientation().Get( axis, angle );

	glPushMatrix();
	glTranslatef( pos[0], pos[1], pos[2] );
	glRotatef( Rad2Deg(angle), axis[0], axis[1], axis[2] );

	glCallList( m_displayList );


	glDisable(GL_LIGHTING);
	glDisable(GL_CULL_FACE);
	glDisable(GL_DEPTH_TEST);

	/*
	for( unsigned int i = 0; i < m_boxes.size(); i++ )
	{
		glPushMatrix();
	// The collision triangle
	if( m_boxes[i].IsLeaf() )
	{
		glBegin(GL_TRIANGLES);
		glColor3f(1,0,0);
			glVertex3fv(m_tris[m_boxes[i].m_iTri].p[0].pointer());
			glVertex3fv(m_tris[m_boxes[i].m_iTri].p[1].pointer());
			glVertex3fv(m_tris[m_boxes[i].m_iTri].p[2].pointer());
		glEnd();
	}

	// Active BBox

	glColor3f(1,1,0);
	float f[16];
	f[0] = m_boxes[i].m_rot[0][0];
	f[1] = m_boxes[i].m_rot[0][1];
	f[2] = m_boxes[i].m_rot[0][2];
	f[3] = 0;			   
	f[4] = m_boxes[i].m_rot[1][0];
	f[5] = m_boxes[i].m_rot[1][1];
	f[6] = m_boxes[i].m_rot[1][2];
	f[7] = 0;			   
	f[8] = m_boxes[i].m_rot[2][0];
	f[9] = m_boxes[i].m_rot[2][1];
	f[10] = m_boxes[i].m_rot[2][2];
	f[11] = 0;
	f[12] = 0;
	f[13] = 0;
	f[14] = 0;
	f[15] = 1;
	glTranslatef(m_boxes[i].m_trans[0],m_boxes[i].m_trans[1],m_boxes[i].m_trans[2]);
	glMultMatrixf( f );
	glBegin(GL_LINE_LOOP);
		glVertex3f(-m_boxes[i].m_ext[0], -m_boxes[i].m_ext[1], -m_boxes[i].m_ext[2]);
		glVertex3f(-m_boxes[i].m_ext[0], -m_boxes[i].m_ext[1], m_boxes[i].m_ext[2]);
		glVertex3f(-m_boxes[i].m_ext[0], m_boxes[i].m_ext[1], m_boxes[i].m_ext[2]);
		glVertex3f(-m_boxes[i].m_ext[0], m_boxes[i].m_ext[1], -m_boxes[i].m_ext[2]);

		glVertex3f(m_boxes[i].m_ext[0], m_boxes[i].m_ext[1], -m_boxes[i].m_ext[2]);
		glVertex3f(m_boxes[i].m_ext[0], m_boxes[i].m_ext[1], m_boxes[i].m_ext[2]);
		glVertex3f(m_boxes[i].m_ext[0], -m_boxes[i].m_ext[1], m_boxes[i].m_ext[2]);
		glVertex3f(m_boxes[i].m_ext[0], -m_boxes[i].m_ext[1], -m_boxes[i].m_ext[2]);
	glEnd();
	glBegin(GL_LINES);
		glVertex3f(-m_boxes[i].m_ext[0], -m_boxes[i].m_ext[1], -m_boxes[i].m_ext[2]);
		glVertex3f(-m_boxes[i].m_ext[0], m_boxes[i].m_ext[1], -m_boxes[i].m_ext[2]);
		glVertex3f(m_boxes[i].m_ext[0], m_boxes[i].m_ext[1], -m_boxes[i].m_ext[2]);
		glVertex3f(m_boxes[i].m_ext[0], -m_boxes[i].m_ext[1], -m_boxes[i].m_ext[2]);

		glVertex3f(-m_boxes[i].m_ext[0], m_boxes[i].m_ext[1], m_boxes[i].m_ext[2]);
		glVertex3f(m_boxes[i].m_ext[0], m_boxes[i].m_ext[1], m_boxes[i].m_ext[2]);
		glVertex3f(m_boxes[i].m_ext[0], -m_boxes[i].m_ext[1], m_boxes[i].m_ext[2]);
		glVertex3f(-m_boxes[i].m_ext[0], -m_boxes[i].m_ext[1], m_boxes[i].m_ext[2]);
	glEnd();
	glPopMatrix();
	} 
	*/
/*
	
	for( unsigned int i = 0; i < bi.size(); i++ )
	{
		glPushMatrix();
	// The collision triangle
	if( m_boxes[bi[i]].IsLeaf() )
	{
		glBegin(GL_TRIANGLES);
		glColor3f(1,0,0);
			glVertex3fv(m_tris[m_boxes[bi[i]].m_iTri].p[0].pointer());
			glVertex3fv(m_tris[m_boxes[bi[i]].m_iTri].p[1].pointer());
			glVertex3fv(m_tris[m_boxes[bi[i]].m_iTri].p[2].pointer());
		glEnd();
	}

	// Active BBox

	glColor3f(1,1,0);
	float f[16];
	f[0] = m_boxes[bi[i]].m_rot[0][0];
	f[1] = m_boxes[bi[i]].m_rot[0][1];
	f[2] = m_boxes[bi[i]].m_rot[0][2];
	f[3] = 0;			   
	f[4] = m_boxes[bi[i]].m_rot[1][0];
	f[5] = m_boxes[bi[i]].m_rot[1][1];
	f[6] = m_boxes[bi[i]].m_rot[1][2];
	f[7] = 0;			   
	f[8] = m_boxes[bi[i]].m_rot[2][0];
	f[9] = m_boxes[bi[i]].m_rot[2][1];
	f[10] = m_boxes[bi[i]].m_rot[2][2];
	f[11] = 0;
	f[12] = 0;
	f[13] = 0;
	f[14] = 0;
	f[15] = 1;
	glTranslatef(m_boxes[bi[i]].m_trans[0],m_boxes[bi[i]].m_trans[1],m_boxes[bi[i]].m_trans[2]);
	glMultMatrixf( f );
	glBegin(GL_LINE_LOOP);
		glVertex3f(-m_boxes[bi[i]].m_ext[0], -m_boxes[bi[i]].m_ext[1], -m_boxes[bi[i]].m_ext[2]);
		glVertex3f(-m_boxes[bi[i]].m_ext[0], -m_boxes[bi[i]].m_ext[1], m_boxes[bi[i]].m_ext[2]);
		glVertex3f(-m_boxes[bi[i]].m_ext[0], m_boxes[bi[i]].m_ext[1], m_boxes[bi[i]].m_ext[2]);
		glVertex3f(-m_boxes[bi[i]].m_ext[0], m_boxes[bi[i]].m_ext[1], -m_boxes[bi[i]].m_ext[2]);

		glVertex3f(m_boxes[bi[i]].m_ext[0], m_boxes[bi[i]].m_ext[1], -m_boxes[bi[i]].m_ext[2]);
		glVertex3f(m_boxes[bi[i]].m_ext[0], m_boxes[bi[i]].m_ext[1], m_boxes[bi[i]].m_ext[2]);
		glVertex3f(m_boxes[bi[i]].m_ext[0], -m_boxes[bi[i]].m_ext[1], m_boxes[bi[i]].m_ext[2]);
		glVertex3f(m_boxes[bi[i]].m_ext[0], -m_boxes[bi[i]].m_ext[1], -m_boxes[bi[i]].m_ext[2]);
	glEnd();
	glBegin(GL_LINES);
		glVertex3f(-m_boxes[bi[i]].m_ext[0], -m_boxes[bi[i]].m_ext[1], -m_boxes[bi[i]].m_ext[2]);
		glVertex3f(-m_boxes[bi[i]].m_ext[0], m_boxes[bi[i]].m_ext[1], -m_boxes[bi[i]].m_ext[2]);
		glVertex3f(m_boxes[bi[i]].m_ext[0], m_boxes[bi[i]].m_ext[1], -m_boxes[bi[i]].m_ext[2]);
		glVertex3f(m_boxes[bi[i]].m_ext[0], -m_boxes[bi[i]].m_ext[1], -m_boxes[bi[i]].m_ext[2]);

		glVertex3f(-m_boxes[bi[i]].m_ext[0], m_boxes[bi[i]].m_ext[1], m_boxes[bi[i]].m_ext[2]);
		glVertex3f(m_boxes[bi[i]].m_ext[0], m_boxes[bi[i]].m_ext[1], m_boxes[bi[i]].m_ext[2]);
		glVertex3f(m_boxes[bi[i]].m_ext[0], -m_boxes[bi[i]].m_ext[1], m_boxes[bi[i]].m_ext[2]);
		glVertex3f(-m_boxes[bi[i]].m_ext[0], -m_boxes[bi[i]].m_ext[1], m_boxes[bi[i]].m_ext[2]);
	glEnd();
	glPopMatrix();
	} 
*/
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_CULL_FACE);
	glEnable(GL_LIGHTING);

	glPopMatrix();

}

PolyMeshBody::~PolyMeshBody()
{
    
}

PolyMeshBody* PolyMeshBody::Clone()
{
    PolyMeshBody* p = new PolyMeshBody(*this);
    return p;
}

} // namespace physics
