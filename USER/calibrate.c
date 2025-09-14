
#define _CALIBRATE_C_



/****************************************************/
/*                                                  */
/* Included files                                   */
/*                                                  */
/****************************************************/

#include "Calibrate.h"



/**********************************************************************
 *
 *     Function: setCalibrationMatrix()
 *
 *  Description: Calling this function with valid input data
 *                in the display and screen input arguments 
 *                causes the calibration factors between the
 *                screen and display points to be calculated,
 *                and the output argument - matrixPtr - to be 
 *                populated.
 *
 *               This function needs to be called only when new
 *                calibration factors are desired.
 *               
 *  
 *  Argument(s): displayPtr (input) - Pointer to an array of three 
 *                                     sample, reference points.
 *               screenPtr (input) - Pointer to the array of touch 
 *                                    screen points corresponding 
 *                                    to the reference display points.
 *               matrixPtr (output) - Pointer to the calibration 
 *                                     matrix computed for the set 
 *                                     of points being provided.
 *

 *       Return: OK - the calibration matrix was correctly 
 *                     calculated and its value is in the 
 *                     output argument.
 *               NOT_OK - an error was detected and the 
 *                         function failed to return a valid
 *                         set of matrix values.
 *                        The only time this sample code returns
 *                        NOT_OK is when Divider == 0
 *

 *
 */
int setCalibrationMatrix( POINT * displayPtr,
                          POINT * screenPtr,
                          MATRIX * matrixPtr)
{

    int  retValue = OK ;


    
    matrixPtr->Divider = ((screenPtr[0].x - screenPtr[2].x) * (screenPtr[1].y - screenPtr[2].y)) - 
                         ((screenPtr[1].x - screenPtr[2].x) * (screenPtr[0].y - screenPtr[2].y)) ;

    if( matrixPtr->Divider == 0 )
    {
        retValue = NOT_OK ;
    }
    else
    {
        matrixPtr->An = ((displayPtr[0].x - displayPtr[2].x) * (screenPtr[1].y - screenPtr[2].y)) - 
                        ((displayPtr[1].x - displayPtr[2].x) * (screenPtr[0].y - screenPtr[2].y)) ;

        matrixPtr->Bn = ((screenPtr[0].x - screenPtr[2].x) * (displayPtr[1].x - displayPtr[2].x)) - 
                        ((displayPtr[0].x - displayPtr[2].x) * (screenPtr[1].x - screenPtr[2].x)) ;

        matrixPtr->Cn = (screenPtr[2].x * displayPtr[1].x - screenPtr[1].x * displayPtr[2].x) * screenPtr[0].y +
                        (screenPtr[0].x * displayPtr[2].x - screenPtr[2].x * displayPtr[0].x) * screenPtr[1].y +
                        (screenPtr[1].x * displayPtr[0].x - screenPtr[0].x * displayPtr[1].x) * screenPtr[2].y ;

        matrixPtr->Dn = ((displayPtr[0].y - displayPtr[2].y) * (screenPtr[1].y - screenPtr[2].y)) - 
                        ((displayPtr[1].y - displayPtr[2].y) * (screenPtr[0].y - screenPtr[2].y)) ;
    
        matrixPtr->En = ((screenPtr[0].x - screenPtr[2].x) * (displayPtr[1].y - displayPtr[2].y)) - 
                        ((displayPtr[0].y - displayPtr[2].y) * (screenPtr[1].x - screenPtr[2].x)) ;

        matrixPtr->Fn = (screenPtr[2].x * displayPtr[1].y - screenPtr[1].x * displayPtr[2].y) * screenPtr[0].y +
                        (screenPtr[0].x * displayPtr[2].y - screenPtr[2].x * displayPtr[0].y) * screenPtr[1].y +
                        (screenPtr[1].x * displayPtr[0].y - screenPtr[0].x * displayPtr[1].y) * screenPtr[2].y ;
    }
 
    return( retValue ) ;

} /* end of setCalibrationMatrix() */



/**********************************************************************
 *
 *     Function: getDisplayPoint()
 *
 *  Description: Given a valid set of calibration factors and a point
 *                value reported by the touch screen, this function
 *                calculates and returns the true (or closest to true)
 *                display point below the spot where the touch screen 
 *                was touched.
 * 
 *
 * 
 *  Argument(s): displayPtr (output) - Pointer to the calculated
 *                                      (true) display point.
 *               screenPtr (input) - Pointer to the reported touch
 *                                    screen point.
 *               matrixPtr (input) - Pointer to calibration factors
 *                                    matrix previously calculated
 *                                    from a call to 
 *                                    setCalibrationMatrix()
 * 
 * 
 *       Return: OK - the display point was correctly calculated 
 *                     and its value is in the output argument.
 *               NOT_OK - an error was detected and the function
 *                         failed to return a valid point.
 *
 *
 *
 *                 NOTE!    NOTE!    NOTE!
 *
 *  setCalibrationMatrix() and getDisplayPoint() will do fine
 *  for you as they are, provided that your digitizer         
 *  resolution does not exceed 10 bits (1024 values).  Higher
 *  resolutions may cause the integer operations to overflow
 *  and return incorrect values.  If you wish to use these   
 *  functions with digitizer resolutions of 12 bits (4096    
 *  values) you will either have to a) use 64-bit signed     
 *  integer variables and math, or b) judiciously modify the 
 *  operations to scale results by a factor of 2 or even 4.  
 *
 *
 */
int getDisplayPoint( POINT * displayPtr,
                     POINT * screenPtr,
                     MATRIX * matrixPtr )
{
    int  retValue = OK ;


    if( matrixPtr->Divider != 0 )
    {

            /* Operation order is important since we are doing integer */
            /*  math. Make sure you add all terms together before      */
            /*  dividing, so that the remainder is not rounded off     */
            /*  prematurely.                                           */

        displayPtr->x = ( (matrixPtr->An * screenPtr->x) + 
                          (matrixPtr->Bn * screenPtr->y) + 
                           matrixPtr->Cn 
                        ) / matrixPtr->Divider ;

        displayPtr->y = ( (matrixPtr->Dn * screenPtr->x) + 
                          (matrixPtr->En * screenPtr->y) + 
                           matrixPtr->Fn 
                        ) / matrixPtr->Divider ;
    }
    else
    {
        retValue = NOT_OK ;
    }

    return( retValue ) ;

} /* end of getDisplayPoint() */

