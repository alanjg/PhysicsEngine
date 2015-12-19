#include "RigidBody.h"
#include "SphereBody.h"
#include "PolyMeshBody.h"
#include "Constants.h"
#include "Collision.h"
#include "Solver.h"
#include "Vector3.h"
#include "Simulation.h"
#include <GL/glut.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>
#include <cstdio>
#include "Timer.h"

#include "ppm.h"

using namespace std;
using namespace physics;

int v_width,v_height;
bool PRINT_FRAME = false;
bool OUTPUT_FILE = false;
float x=0;
float y=0;
float z=30;
float theta = 45;
float phi = 15;
float scale = 1;
SphereBody* p;
Simulation sim;
bool pause = true;

void DrawQuad(Vector3 a,Vector3 b,Vector3 c,Vector3 d)
{
	glBegin(GL_QUADS);
	Vector3 n=(b-a).Cross(d-a);
	n.Normalize();
	glNormal3f(n[0],n[1],n[2]);
	glVertex3fv(a.pointer());
	glVertex3fv(b.pointer());
	glVertex3fv(c.pointer());
	glVertex3fv(d.pointer());
	glEnd();
}
void DisplayFunc()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	glTranslatef(-x,-y,-z);
	glRotatef(phi,1,0,0);
	glRotatef(theta,0,1,0);
    //gluLookAt(0,0,100,0,0,0,0,1,0);
	
	Vector3 r(30,0,0);
	Vector3 u(0,30,0);
	Vector3 o(0,0,30);

//	DrawQuad(r+u-o,-r+u-o,-r-u-o,r-u-o);
//	DrawQuad(r+u-o,r-u-o,r-u+o,r+u+o);
//	DrawQuad(r+u+o,r-u+o,-r-u+o,-r+u+o);
//	DrawQuad(-r+u+o,-r-u+o,-r-u-o,-r+u-o);
//	DrawQuad(r-u+o,r-u-o,-r-u-o,-r-u+o);

	static Timer t;
	float now = t.GetElapsedTime();
	if(!pause)
	{
		sim.Update(now);
	}
	t.Reset();
	glColor3f(1,1,1);
	sim.Render();

	glutSwapBuffers();
	glutPostRedisplay();

	
	if(OUTPUT_FILE || PRINT_FRAME)
	{
		PRINT_FRAME = false;
		static int frame = 0;
		ostringstream out;
		out << "movie/physics" << frame << ".ppm";
		frame++;
		FILE* fp = fopen(out.str().c_str(),"w");
		
		DumpPPM(fp,0,v_width,v_height);
		fclose(fp);
	}
}

void ReshapeFunc(int width, int height)
{
	v_width = width;
	v_height = height;
	if(height==0)
		height=1;
	
    glViewport(0,0,width,height);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45,float(width)/float(height),1,1000);
	glMatrixMode(GL_MODELVIEW);
}

void KeyboardFunc(unsigned char key,int /*mx*/,int /*my*/)
{
	switch(key)
	{
	case 'a':  //MOVE LEFT
		x-=0.1f;	
		break;

	case 'd': //MOVE RIGHT
		x+=0.1f;	
		break;

	case 'w': //MOVE UP
		y+=0.1f;	
		break;

	case 's': //MOVE DOWN
		y-=0.1f;
		break;

	case 'q': //ROTATE CCW
		theta+=2;
		break;

	case 'e': //ROTATE CW
		theta-=2;
		break;
	case 'r':
		phi +=2;
		break;

	case 'f':
		phi -=2;
		break;
	case 'p':
		pause = !pause;
		break;

	case ' ':
		sim.AddObject(new SphereBody(Vector3(0,10,0),1.5,true));
		break;

	case 'l':
		PRINT_FRAME = true;
		break;
    }
}

void InitGL(int argc,char **argv)
{
	v_width = 800;
	v_height = 600;
	glutInit(&argc,argv);

	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA); 
	glutInitWindowSize(v_width,v_height); 
	glutInitWindowPosition(0,0); 
	glutCreateWindow("Physics"); 

	glutDisplayFunc(DisplayFunc); 
	glutReshapeFunc(ReshapeFunc);
	glutKeyboardFunc(KeyboardFunc);

	//glPolygonMode(GL_FRONT,GL_LINE);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_CULL_FACE);
	float pos[4]={0,0,10,1};
	glLightfv(GL_LIGHT0,GL_POSITION,pos);


	glEnable(GL_VERTEX_ARRAY);
	glEnable(GL_NORMAL_ARRAY);
}

void tester()
{
	RigidBody *b1,*b2;	

}


int main(int argc,char **argv)
{
	InitGL( argc, argv );

	string sceneFileName;
	string token;
	ifstream sceneFile;

	// Get the name of the scene file from the user
	cout << "Scene file: ";
	cin >> sceneFileName;
//	sceneFileName = "scene1.scn";

	sceneFile.open( sceneFileName.c_str() );
	getline( sceneFile, token );
	while( token != "##Physics Simulator Scene Description File##" )
	{
		sceneFile.close();
		cout << "Invalid file. Try again." << endl;
		cin >> sceneFileName;
		sceneFile.open( sceneFileName.c_str() );
		getline( sceneFile, token );
	}

	// Read camera position
	sceneFile >> token;
	if( token != "Camera" )
	{
		sceneFile.close();
		cerr << "Invalid scene description file." << endl;
		exit(1);
	}
	sceneFile >> x >> y >> z >> theta >> phi;

	Vector3 pos;
	Vector3 rotAxis;
	float   rotAngle;
	bool    bMobile;

	// Read each object file. They have to be in the same directory as the scene file.
	ifstream objectFile;
	while( sceneFile >> token )
	{
		if(token[0] == '#')
		{
			getline(sceneFile,token);
		}
		else if(token == "polymesh")
		{
			sceneFile >> token;
			sceneFile >> pos[0] >> pos[1] >> pos[2] >> rotAxis[0] >> rotAxis[1] >> rotAxis[2] >> rotAngle >> bMobile;
			objectFile.open( token.c_str(), ios_base::in | ios_base::binary );
			PolyMeshBody* mesh = new PolyMeshBody( objectFile, pos, Rotation(rotAxis, Deg2Rad(rotAngle)), bMobile );
			objectFile.close();
			sim.AddObject( mesh );
		}
		else if(token == "sphere")
		{
			sceneFile >> pos[0] >> pos[1] >> pos[2] >> rotAngle;
			SphereBody* sphere = new SphereBody(pos,rotAngle,true);
			sim.AddObject(sphere);
		}
		else
		{
			getline(sceneFile,token);
		}
	}
	sceneFile.close();

	glutMainLoop();

	return 0;
}
