//*****************************************************************************
//
//				Stellaris IMU (FREEIMU PORT)
//
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
//
//		Author 				: Terence Ang - terenceang@mac.com
//		Original Code 		:  Hari Nair- hair.nair@gmail.com
//		Original i2c code 	: JOERG QUINTEN (aBUGSworstnightmare)
//		Original ftoa code	: JOERG QUINTEN (aBUGSworstnightmare)
//
//					1st Draft .. 09/March/2013
//
//*****************************************************************************
#include <stdint.h>
#include <stdbool.h>
#include "inc/common.h"
#include "driverlib/i2c.h"
#include <math.h>
#include "inc/tmrsys.h"
#include "inc/adxl345.h"
#include "inc/hmc5883l.h"
#include "inc/l3g4200d.h"



void util_BtnClear(void) {
    gbBtnPressed = 0;
    }


void util_MemSet(u08* pBuffer, u08 val, int nBytes ) {
    while (nBytes--) {
        *pBuffer++ = val;
        }
    }

///
/// Copies one block of memory in RAM to another.
///
/// @param pDestination The destination block of memory
/// @param pSource The source block of memory
/// @param nBytes The total number of bytes to copy
void util_MemCpy(u08* pDestination, u08* pSource, int nBytes) {
    while (nBytes--) {
    	*pDestination++ = *pSource++;
	}
    }

int util_AverageSamples(short buf[], char numSamples) {
    int cnt, average;
    cnt = numSamples;
    average = 0;
    while (cnt--) {
        average += buf[cnt];
        }
    average +=  ((average < 0)? -numSamples/2 : numSamples/2);
    average /= numSamples;
    return average;
    }

int util_SigmaSamples(short buf[], char numSamples, int average) {
    int cnt, variance,tmp;
    double sigma;
    cnt = numSamples;
    variance = 0;
    while (cnt--)  {
        tmp = buf[cnt] - average;
        variance += (tmp*tmp);
        }
    sigma = sqrt((double)variance/ (double)(numSamples-1));
    return (int)(sigma+0.5);
    }
