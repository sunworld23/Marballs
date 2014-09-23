/*************************************************************
 * decimal_precision.h
 * -------------
 * Header file that defines the engine's decimal precision.
 *
 * Last Revision: Sept. 22, 2014
 *
 * TO DO: - Continue following tutorial to fill this out?
 *		  - This bullet included for to do list formatting.
 *************************************************************/

/********************************************************
 * #include guards to protect against double inclusions
 * when using header files
*********************************************************/
#ifndef DECIMAL_PRECISION_INCLUDED
#define DECIMAL_PRECISION_INCLUDED

/**
 * To allow the marballs engine to handle both single and double precision
 * we must tell the preprocessor to use either the float version of function
 * calls or the double version of function calls.
**/

#include <float.h> //Needed to redefine the DBL_MAX and FLT_MAX

namespace marballs
{

    /**
     * Depending on the different mode type we need to redefine various
     * functions and constants. This lets the engine switch between single
     * and double precision easier.
    **/
    
    #if 0
        #define SINGLE_PRECISION
        typedef float marb;
        #define MARB_MAX        FLT_MAX
        #define MARB_MIN        FLT_MIN
        #define MARB_EPSILON    FLT_EPSILON
        #define MARB_PI         3.14159f
        #define marb_sin        sinf
        #define marb_cos        cosf
        #define marb_tan        tanf
        #define marb_exp        expf
        #define marb_pow        powf
        #define marb_sqrt       sqrtf
        #define marb_abs        fabs
        #define marb_fmod       fmodf
    #else
        #define DOUBLE_PRECISION
        typedef double marb;
        #define MARB_MAX        DBL_MAX
        #define MARB_MIN        DBL_MIN
        #define MARB_EPSILON    DBL_EPSILON
        #define MARB_PI         3.14159265358979
        #define marb_sin        sin
        #define marb_cos        cos
        #define marb_tan        tan
        #define marb_exp        exp
        #define marb_pow        pow
        #define marb_sqrt       sqrt
        #define marb_abs        fabs
        #define marb_fmod       fmod
    #endif
}

#endif //DECIMAL_PRECISION_INCLUDED
