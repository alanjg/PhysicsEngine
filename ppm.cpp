#include "ppm.h"
#include <cstdio>
#include <gl/glut.h>

using namespace std;
// dumps a PPM raw (P6) file
void DumpPPM(FILE *fp, int buffer,int width,int height)
{
    const int maxVal=255;
    register int y;
    unsigned char *pixels;

    
    
    fprintf(fp,	"P6 ");
    fprintf(fp,	"%d %d ", width, height);
    fprintf(fp,	"%d\n",	maxVal);

    pixels = new unsigned char [3*width];
    if(	pixels == NULL )
    {
	fprintf(stderr,"Cannot allocate	enough memory\n") ;
	return ;
    }

    if( buffer == 0 )
	glReadBuffer(GL_FRONT) ;
    else
	glReadBuffer(GL_BACK) ;
    for	( y = height-1;	y>=0; y-- ) {
	glReadPixels(0,y,width,1,GL_RGB,GL_UNSIGNED_BYTE, (GLvoid *) pixels); 
#ifdef WIN32
	//  RGB <-> GBR for windows (beats me why!)
	unsigned char *p = &pixels[0] ;
	for( int i = 0 ; i < 3*width ; i += 3 )
	{
	    unsigned char v[3] ;
	    v[0] = p[i+1] ;
	    v[1] = p[i+2] ;
	    v[2] = p[i] ;
	    fwrite(v, 3*sizeof(unsigned char), 1, fp);
	}
#else
	fwrite(pixels, 3, width, fp);
#endif
    }
    delete [] pixels;
}