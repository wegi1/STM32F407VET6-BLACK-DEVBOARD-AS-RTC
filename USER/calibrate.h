
#ifndef _CALIBRATE_H_

#define _CALIBRATE_H_

/****************************************************/
/*                                                  */
/* Included files                                   */
/*                                                  */
/****************************************************/

#include <math.h>
#include "inttypes.h"

/****************************************************/
/*                                                  */
/* Definitions                                      */
/*                                                  */
/****************************************************/

#ifndef		_CALIBRATE_C_
	#define		EXTERN         extern
#else
	#define		EXTERN
#endif



#ifndef		OK
	#define		OK		        0
	#define		NOT_OK		   -1
#endif



#define			INT32				long

/****************************************************/
/*                                                  */
/* Structures                                       */
/*                                                  */
/****************************************************/


typedef struct Point {
                        uint16_t    x,
                                 y ;
                     } POINT ;



typedef struct Matrix {
							/* This arrangement of values facilitates 
							 *  calculations within getDisplayPoint() 
							 */
                        INT32    An,     /* A = An/Divider */
                                 Bn,     /* B = Bn/Divider */
                                 Cn,     /* C = Cn/Divider */
                                 Dn,     /* D = Dn/Divider */
                                 En,     /* E = En/Divider */
                                 Fn,     /* F = Fn/Divider */
                                 Divider ;
                     } MATRIX ;

/****************************************************/
/*                                                  */
/* Function declarations                            */
/*                                                  */
/****************************************************/


EXTERN int setCalibrationMatrix( POINT * display,
                                 POINT * screen,
                                 MATRIX * matrix) ;


EXTERN int getDisplayPoint( POINT * display,
                            POINT * screen,
                            MATRIX * matrix ) ;


#endif  /* _CALIBRATE_H_ */
