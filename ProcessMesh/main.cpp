//-----------------------------------------------------------------------------
// main.cpp
//
// Main entry point for the ProcessMesh program, used to generate OBB trees
// and other miscellaneous preprocessing that needs to be done for meshes
//-----------------------------------------------------------------------------

#define DOUBLE_PRECISION

#include "geom.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>

using namespace std;
using namespace physics;


extern vector<Tri> tris;
extern unsigned int numtris;

bool BuildBBoxTree();
bool WriteBBoxTree( ofstream& outStream );


bool ParseXFile( ifstream& infile, ofstream& outfile )
{
	string line;
	char c;

	getline( infile, line );
	while( line.find("Mesh {") == string::npos )
	{
		getline( infile, line );
	}

	unsigned int numVerts;
	infile >> numVerts >> c;

	vector<float> f(3);
	vector< vector<float> > verts(numVerts, f);
	vector< vector<float> > normals(numVerts, f);
	f.resize(2);
	vector< vector<float> > texCoords(numVerts, f);

	for( unsigned int i = 0; i < numVerts; i++ )
	{
		infile >> verts[i][0] >> c >> verts[i][1] >> c >> verts[i][2] >> c >> c;
	}

	infile >> numtris >> c;
	if( !infile )
	{
		infile.close();
		return false;
	}
	tris.resize( numtris );
	vector<unsigned int> x(3);
	vector< vector<unsigned int> > indices(numtris, x);
	unsigned int numFaceSides;
	unsigned int index;
	for( unsigned int i = 0; i < numtris; i++ )
	{
		infile >> numFaceSides >> c;
		if( numFaceSides != 3 )
		{
			cerr << "Error: input mesh contains faces with more than 3 sides" << endl;
			infile.close();
			return false;
		}

		infile >> index >> c;
		indices[i][0] = index;
		tris[i].p[0].Set( verts[index][0], verts[index][1], verts[index][2] );
		infile >> index >> c;
		indices[i][1] = index;
		tris[i].p[1].Set( verts[index][0], verts[index][1], verts[index][2] );
		infile >> index >> c;
		indices[i][2] = index;
		tris[i].p[2].Set( verts[index][0], verts[index][1], verts[index][2] );
		infile >> c;
	}

	infile >> line >> c;
	if( line != "MeshNormals" )
	{
		cerr << "Error: input mesh format is invalid" << endl;
		infile.close();
		return false;
	}

	unsigned int numNormals;
	infile >> numNormals >> c;
	if( numNormals != numVerts )
	{
		cerr << "Error: input mesh format is invalid" << endl;
		infile.close();
		return false;
	}

	for( unsigned int i = 0; i < numNormals; i++ )
	{
		infile >> normals[i][0] >> c >> normals[i][1] >> c >> normals[i][2] >> c >> c;
	}

	unsigned int numNormTris;
	infile >> numNormTris >> c;
	if( numNormTris != numtris )
	{
		cerr << "Error: input mesh format is invalid" << endl;
		infile.close();
		return false;
	}

	unsigned int numTexCoords = 0;
	getline( infile, line );
	while( line.find('}') == string::npos )
	{
		getline( infile, line );
	}
	infile >> line >> c;
	if( line == "MeshTextureCoords" )
	{
		infile >> numTexCoords >> c;
		if( numTexCoords != numVerts )
		{
			cerr << "Error: input mesh format is invalid" << endl;
			infile.close();
			return false;
		}

		for( unsigned int i = 0; i < numTexCoords; i++ )
		{
			infile >> texCoords[i][0] >> c >> texCoords[i][1] >> c >> c;
			if( !infile )
			{
				infile.close();
				return false;
			}
		}
	}

	if( !infile )
	{
		infile.close();
		return false;
	}

	infile.close();

	unsigned int numIndices = numtris * 3;
	outfile.write( (char*)&numIndices, sizeof(unsigned int) );
	outfile.write( (char*)&numVerts, sizeof(unsigned int) );
	for( unsigned int i = 0; i < numtris; i++ )
	{
		outfile.write( (char*)&indices[i][0], sizeof(unsigned int) );
		outfile.write( (char*)&indices[i][1], sizeof(unsigned int) );
		outfile.write( (char*)&indices[i][2], sizeof(unsigned int) );
	}
	for( unsigned int i = 0; i < numVerts; i++ )
	{
		outfile.write( (char*)&verts[i][0], sizeof(float) );
		outfile.write( (char*)&verts[i][1], sizeof(float) );
		outfile.write( (char*)&verts[i][2], sizeof(float) );
	}
	for( unsigned int i = 0; i < numNormals; i++ )
	{
		outfile.write( (char*)&normals[i][0], sizeof(float) );
		outfile.write( (char*)&normals[i][1], sizeof(float) );
		outfile.write( (char*)&normals[i][2], sizeof(float) );
	}
	for( unsigned int i = 0; i < numTexCoords; i++ )
	{
		outfile.write( (char*)&texCoords[i][0], sizeof(float) );
		outfile.write( (char*)&texCoords[i][1], sizeof(float) );
	}

	unsigned char term = 0xff;
	outfile.write( (char*)&term, 1 );

	return true;
}

int main( int argc, char ** argv )
{
	if( argc != 5 && argc != 6 )
	{
		cout << "Usage: ProcessMesh <infilename> (<outfilename>) <mass> <bounciness> <rotational inertia>\n";
		exit(1);
	}

	string infilename = argv[1];
	if( infilename.substr(infilename.rfind('.')) != ".x" )
	{
		cerr << "Error: input file must be a .x file\n";
		exit(1);
	}

	ifstream infile( argv[1], ios_base::in | ios_base::binary );
	if( !infile )
	{
		cerr << "Error: cannot find " << argv[1] << " for reading\n";
		exit(1);
	}

	string outfilename;
	if( argc >= 3 )
	{
		outfilename = argv[2];
	}
	else
	{
		outfilename = infilename.substr( 0, infilename.rfind('.') );
		outfilename += ".amf";   // amf = ajg mesh file
	}

	ofstream outfile( outfilename.c_str(), ios_base::out | ios_base::binary );
	if( !outfile )
	{
		cerr << "Error: cannot open " << outfilename << "for writing\n";
		infile.close();
		exit(1);
	}

	const char* header = "amf";
	outfile.write( header, 3 );

	// read input mesh
	if( !ParseXFile(infile, outfile) )
	{
		cerr << "Error while reading from " << argv[1] << endl;
		infile.close();
		outfile.close();
		exit(1);
	}

	if( BuildBBoxTree() )
	{
		if( !WriteBBoxTree(outfile) )
		{
			cerr << "Error while writing to " << outfilename << endl;
			infile.close();
			outfile.close();
			exit(1);
		}
	}

	int i = (argc == 5) ? 2 : 3;
	float invMass = (float)(1.0 / atof( argv[i] ));
	float bounciness = (float)atof( argv[i+1] );
	float inertia = (float)atof( argv[i+2] );
	outfile.write( (char*)&invMass, sizeof(float) );
	outfile.write( (char*)&bounciness, sizeof(float) );
	outfile.write( (char*)&inertia, sizeof(float) );

	infile.close();
	outfile.close();

	return 0;
}
