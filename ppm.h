#ifndef PPM_H
#define PPM_H

#include <cstdio>

// dumps a PPM raw (P6) file
void DumpPPM(std::FILE *fp, int buffer,int width,int height);

#endif