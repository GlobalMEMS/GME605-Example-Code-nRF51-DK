#include "math.h"
#include "AKFS_Device.h"
#include "ossf_magnetic.h"

AKLIB_C_API_START

// function sets the 3x3 matrix A to the identity matrix
static void f3x3matrixAeqI(float A[][3])
{
  float *pAij;	// pointer to A[i][j]
  int8 i, j;		// loop counters

  for (i = 0; i < 3; i++)
    {
      // set pAij to &A[i][j=0]
      pAij = A[i];
      for (j = 0; j < 3; j++)
	{
	  *(pAij++) = 0.0F;
	}
      A[i][i] = 1.0F;
    }
  return;
}

// function sets the matrix A to the identity matrix
static void fmatrixAeqI(float *A[], int16 rc)
{
  // rc = rows and columns in A

  float *pAij;	// pointer to A[i][j]
  int8 i, j;		// loop counters

  for (i = 0; i < rc; i++)
    {
      // set pAij to &A[i][j=0]
      pAij = A[i];
      for (j = 0; j < rc; j++)
	{
	  *(pAij++) = 0.0F;
	}
      A[i][i] = 1.0F;
    }
  return;
}

// function uses Gauss-Jordan elimination to compute the inverse of matrix A in situ
// on exit, A is replaced with its inverse
static void fmatrixAeqInvA(float *A[],
			   int8 iColInd[],
			   int8 iRowInd[],
			   int8 iPivot[],
			   int8 isize,
			   int8 *pierror)
{
  float largest;					// largest element used for pivoting
  float scaling;					// scaling factor in pivoting
  float recippiv;					// reciprocal of pivot element
  float ftmp;						// temporary variable used in swaps
  int8 i, j, k, l, m;				// index counters
  int8 iPivotRow, iPivotCol;		// row and column of pivot element

  // to avoid compiler warnings
  iPivotRow = iPivotCol = 0;

  // default to successful inversion
  *pierror = 0;//false;

  // initialize the pivot array to 0
  for (j = 0; j < isize; j++)
    {
      iPivot[j] = 0;
    }

  // main loop i over the dimensions of the square matrix A
  for (i = 0; i < isize; i++)
    {
      // zero the largest element found for pivoting
      largest = 0.0F;
      // loop over candidate rows j
      for (j = 0; j < isize; j++)
	{
	  // check if row j has been previously pivoted
	  if (iPivot[j] != 1)
	    {
	      // loop over candidate columns k
	      for (k = 0; k < isize; k++)
		{
		  // check if column k has previously been pivoted
		  if (iPivot[k] == 0)
		    {
		      // check if the pivot element is the largest found so far
		      if (fabsf(A[j][k]) >= largest)
			{
			  // and store this location as the current best candidate for pivoting
			  iPivotRow = j;
			  iPivotCol = k;
			  largest = (float)fabsf(A[iPivotRow][iPivotCol]);
			}
		    }
		  else if (iPivot[k] > 1)
		    {
		      // zero determinant situation: exit with identity matrix and set error flag
		      fmatrixAeqI(A, isize);
		      *pierror = 1; //true;
		      return;
		    }
		}
	    }
	}
      // increment the entry in iPivot to denote it has been selected for pivoting
      iPivot[iPivotCol]++;

      // check the pivot rows iPivotRow and iPivotCol are not the same before swapping
      if (iPivotRow != iPivotCol)
	{
	  // loop over columns l
	  for (l = 0; l < isize; l++)
	    {
	      // and swap all elements of rows iPivotRow and iPivotCol
	      ftmp = A[iPivotRow][l];
	      A[iPivotRow][l] = A[iPivotCol][l];
	      A[iPivotCol][l] = ftmp;
	    }
	}

      // record that on the i-th iteration rows iPivotRow and iPivotCol were swapped
      iRowInd[i] = iPivotRow;
      iColInd[i] = iPivotCol;

      // check for zero on-diagonal element (singular matrix) and return with identity matrix if detected
      if (A[iPivotCol][iPivotCol] == 0.0F)
	{
	  // zero determinant situation: exit with identity matrix and set error flag
	  fmatrixAeqI(A, isize);
	  *pierror = 1;//true;
	  return;
	}

      // calculate the reciprocal of the pivot element knowing it's non-zero
      recippiv = 1.0F / A[iPivotCol][iPivotCol];
      // by definition, the diagonal element normalizes to 1
      A[iPivotCol][iPivotCol] = 1.0F;
      // multiply all of row iPivotCol by the reciprocal of the pivot element including the diagonal element
      // the diagonal element A[iPivotCol][iPivotCol] now has value equal to the reciprocal of its previous value
      for (l = 0; l < isize; l++)
	{
	  if (A[iPivotCol][l] != 0.0F)
	    A[iPivotCol][l] *= recippiv;
	}
      // loop over all rows m of A
      for (m = 0; m < isize; m++)
	{
	  if (m != iPivotCol)
	    {
	      // scaling factor for this row m is in column iPivotCol
	      scaling = A[m][iPivotCol];
	      // zero this element
	      A[m][iPivotCol] = 0.0F;
	      // loop over all columns l of A and perform elimination
	      for (l = 0; l < isize; l++)
		{
		  if ((A[iPivotCol][l] != 0.0F) && (scaling != 0.0F))
		    A[m][l] -= A[iPivotCol][l] * scaling;
		}
	    }
	}
    } // end of loop i over the matrix dimensions

  // finally, loop in inverse order to apply the missing column swaps
  for (l = isize - 1; l >= 0; l--)
    {
      // set i and j to the two columns to be swapped
      i = iRowInd[l];
      j = iColInd[l];

      // check that the two columns i and j to be swapped are not the same
      if (i != j)
	{
	  // loop over all rows k to swap columns i and j of A
	  for (k = 0; k < isize; k++)
	    {
	      ftmp = A[k][i];
	      A[k][i] = A[k][j];
	      A[k][j] = ftmp;
	    }
	}
    }

  return;
}

// ossf calibraton
void ossfCalibration(const AKFVEC hbuf[],	/* datas buffer */
		     const int16	hbuf_n,		/* number of datas in buffer*/
		     AKFVEC *center,			/* center of sphere */
		     AKFLOAT *r,				/* radius */
		     AKFLOAT *fFitErrorpc	/* fit error */)
{
  // local variables
  float fBs2;								// fBs[CHX]^2+fBs[CHY]^2+fBs[CHZ]^2
  float fSumBs4;							// sum of fBs2
  float fscaling;							// set to FUTPERCOUNT * FMATRIXSCALING
  float fE;								// error function = r^T.r
  int16 iOffset[3];						// offset to remove large DC hard iron bias in matrix
  int16 iCount;							// number of measurements counted
  int8 ierror;							// matrix inversion error flag
  int8 i, j, k, l;						// loop counters

  // working arrays for 4x4 matrix inversion
  float *pfRows[4];
  int8 iColInd[4];
  int8 iRowInd[4];
  int8 iPivot[4];

  //Additional
  float ftrinvW[3][3]; // trial inverse soft iron matrix size
  float fvecA[10];				// scratch 10x1 vector used by calibration algorithms
  float fvecB[4];					// scratch 4x1 vector used by calibration algorithms
  float fmatA[4][4];			// scratch 4x4 matrix used by calibration algorithms
  float fmatB[10][10];			// scratch 10x10 matrix used by calibration algorithms
  float ftrV[3];					// trial value of hard iron offset z, y, z (uT)
  float ftrB;						// trial value of geomagnetic field magnitude in uT

  // compute fscaling to reduce multiplications later
  fscaling = 1 / DEFAULTB;

  // the trial inverse soft iron matrix invW always equals the identity matrix for 4 element calibration
  f3x3matrixAeqI(ftrinvW);  //f3x3matrixAeqI(pthisMagCal->ftrinvW);

  // zero fSumBs4=Y^T.Y, fvecB=X^T.Y (4x1) and on and above diagonal elements of fmatA=X^T*X (4x4)
  fSumBs4 = 0.0F;
  for (i = 0; i < 4; i++)
    {
      fvecB[i] = 0.0F;
      for (j = i; j < 4; j++)
	{
	  fmatA[i][j] = 0.0F;
	}
    }

  // the offsets are guaranteed to be set from the first element but to avoid compiler error
  iOffset[CHX] = iOffset[CHY] = iOffset[CHZ] = 0;

  // use from MINEQUATIONS up to MAXEQUATIONS entries from magnetic buffer to compute matrices
  iCount = 0;

  for (k = 0; k < hbuf_n; k++)
    {

      // use first valid magnetic buffer entry as estimate (in counts) for offset
      if (iCount == 0)
	{
	  for (l = CHX; l <= CHZ; l++)
	    {
	      iOffset[l] = hbuf[k].v[l];
	    }
	}

      // store scaled and offset fBs[XYZ] in fvecA[0-2] and fBs[XYZ]^2 in fvecA[3-5]
      for (l = CHX; l <= CHZ; l++)
	{
	  fvecA[l] = (float)((int32)hbuf[k].v[l] - (int32)iOffset[l]) * fscaling;
	  fvecA[l + 3] = fvecA[l] * fvecA[l];
	}

      // calculate fBs2 = fBs[CHX]^2 + fBs[CHY]^2 + fBs[CHZ]^2 (scaled uT^2) 
      fBs2 = fvecA[3] + fvecA[4] + fvecA[5];

      // accumulate fBs^4 over all measurements into fSumBs4=Y^T.Y
      fSumBs4 += fBs2 * fBs2;

      // now we have fBs2, accumulate fvecB[0-2] = X^T.Y =sum(fBs2.fBs[XYZ])
      for (l = CHX; l <= CHZ; l++)
	{
	  fvecB[l] += fvecA[l] * fBs2;
	}

      //accumulate fvecB[3] = X^T.Y =sum(fBs2)
      fvecB[3] += fBs2;

      // accumulate on and above-diagonal terms of fmatA = X^T.X ignoring fmatA[3][3]
      fmatA[0][0] += fvecA[CHX + 3];
      fmatA[0][1] += fvecA[CHX] * fvecA[CHY];
      fmatA[0][2] += fvecA[CHX] * fvecA[CHZ];
      fmatA[0][3] += fvecA[CHX];
      fmatA[1][1] += fvecA[CHY + 3];
      fmatA[1][2] += fvecA[CHY] * fvecA[CHZ];
      fmatA[1][3] += fvecA[CHY];
      fmatA[2][2] += fvecA[CHZ + 3];
      fmatA[2][3] += fvecA[CHZ];

      // increment the counter for next iteration
      iCount++;

    }


  // set the last element of the measurement matrix to the number of buffer elements used
  fmatA[3][3] = (float)iCount;

  // store the number of measurements accumulated (defensive programming, should never be needed)
  //pthisMagBuffer->iMagBufferCount = iCount;

  // use above diagonal elements of symmetric fmatA to set both fmatB and fmatA to X^T.X
  for (i = 0; i < 4; i++)
    {
      for (j = i; j < 4; j++)
	{
	  fmatB[i][j] = fmatB[j][i] = fmatA[j][i] = fmatA[i][j];
	}
    }

  // calculate in situ inverse of fmatB = inv(X^T.X) (4x4) while fmatA still holds X^T.X
  for (i = 0; i < 4; i++)
    {
      pfRows[i] = fmatB[i];
    }
  fmatrixAeqInvA(pfRows, iColInd, iRowInd, iPivot, 4, &ierror);

  // calculate fvecA = solution beta (4x1) = inv(X^T.X).X^T.Y = fmatB * fvecB
  for (i = 0; i < 4; i++)
    {
      fvecA[i] = 0.0F;
      for (k = 0; k < 4; k++)
	{
	  fvecA[i] += fmatB[i][k] * fvecB[k];
	}
    }

  // calculate P = r^T.r = Y^T.Y - 2 * beta^T.(X^T.Y) + beta^T.(X^T.X).beta
  // = fSumBs4 - 2 * fvecA^T.fvecB + fvecA^T.fmatA.fvecA
  // first set P = Y^T.Y - 2 * beta^T.(X^T.Y) = fSumBs4 - 2 * fvecA^T.fvecB
  fE = 0.0F;
  for (i = 0; i < 4; i++)
    {
      fE += fvecA[i] * fvecB[i];
    }
  fE = fSumBs4 - 2.0F * fE;

  // set fvecB = (X^T.X).beta = fmatA.fvecA	
  for (i = 0; i < 4; i++)
    {
      fvecB[i] = 0.0F;
      for (k = 0; k < 4; k++)
	{
	  fvecB[i] += fmatA[i][k] * fvecA[k];
	}
    }

  // complete calculation of P by adding beta^T.(X^T.X).beta = fvecA^T * fvecB
  for (i = 0; i < 4; i++)
    {
      fE += fvecB[i] * fvecA[i];
    }

  // compute the hard iron vector (in uT but offset and scaled by FMATRIXSCALING)
  for (l = CHX; l <= CHZ; l++)
    {
      ftrV[l] = 0.5F * fvecA[l];
    }

  // compute the scaled geomagnetic field strength B (in uT but scaled by FMATRIXSCALING)
  ftrB = sqrtf(fvecA[3] + ftrV[CHX] * ftrV[CHX] +
	       ftrV[CHY] * ftrV[CHY] + ftrV[CHZ] * ftrV[CHZ]);

  // calculate the trial fit error (percent) normalized to number of measurements and scaled geomagnetic field strength
  *fFitErrorpc = sqrtf(fE / (float)iCount) * 100.0F /
    (2.0F * ftrB * ftrB);

  // correct the hard iron estimate for FMATRIXSCALING and the offsets applied (result in uT)
  for (l = CHX; l <= CHZ; l++)
    {
      ftrV[l] = ftrV[l] * DEFAULTB + (float)iOffset[l]; // *AKM_SENSITIVITY;
      center->v[l] = ftrV[l];  //Accept the trial center
    }

  // correct the geomagnetic field strength B to correct scaling (result in uT)
  ftrB *= DEFAULTB;
  *r = ftrB;   //Accept the trial radius

  return;
}

void soft_iron_compensation(AKFVEC* phdata)
{
  /* Soft Iron Matrix Compensation*/
  int i, j;
  AKFVEC tmp_val = {0.f, 0.f, 0.f};
  tmp_val.u.x = phdata->u.x;
  tmp_val.u.y = phdata->u.y;
  tmp_val.u.z = phdata->u.z;
  phdata->u.x = 0;
  phdata->u.y = 0;
  phdata->u.z = 0;


  for (i = 0; i < 3; ++i)
    for (j = 0; j < 3; ++j)
      phdata->v[i] += tmp_val.v[j] * SOFT_IRON_MATRIX[i][j];

  return;
}

AKLIB_C_API_END
