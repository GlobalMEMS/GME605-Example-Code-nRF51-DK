#ifndef OSSF_MAGNETIC_H
#define OSSF_MAGNETIC_H

#include "AKFS_Common.h"

typedef signed long int int32;
#define CHX 0				// X-axis components
#define CHY 1				// Y-axis components
#define CHZ 2				// Z-axis components
#define DEFAULTB 50.0F		// default geomagnetic field (uT)

#define OSSF_HBUF_SIZE (100)        //HBUF size for the OSSF algorithm
#define OSSF_CLEAR_BUF_SIZE (0)     //No HBUF clean-up after a successful calibraion
#define OSSF_HBUF_GET4PTS_SIZE (10) //Number of points to use for the 4-pts algorithm

extern float SOFT_IRON_MATRIX[][3]; //Soft iron matrix

AKLIB_C_API_START

void ossfCalibration(
		     const AKFVEC hbuf[],	/* datas buffer */
		     const int16	hbuf_n,		/* number of datas in buffer*/
		     AKFVEC *center,			/* center of sphere */
		     AKFLOAT *r,				/* radius */
		     AKFLOAT *fFitErrorpc	/* fit error */);

void soft_iron_compensation(AKFVEC* phdata);

AKLIB_C_API_END

#endif   // #ifndef OSSF_MAGNETIC_H
