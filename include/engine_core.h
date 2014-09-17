/********************************************************
 * #include guards to protect against double inclusions
 * when using header files
*********************************************************/
#ifndef ENGINE_CORE_INCLUDED
#define ENGINE_CORE_INCLUDED

#include <math.h>
#include "decimal_precision.h"

namespace marballs
{
    /**
     * Holds the values for a 3D vector
    **/
    class vector3
    {
        /*********************************
         *     Variable Declarations
        *********************************/
        public:
        
            /**
             * Holds the x-axis value
            **/
            marb x;
            
             /**
             * Holds the y-axis value
            **/
            marb y;
            
             /**
             * Holds the x value
            **/
            marb z;
        
        private:
        
            /**
             * From book:
             *  For performance purposes, some machines run faster
             *  with 4 floating point numbers than three. Padding variable.
            **/
            marb pad;
        
        /*********************************
         *     Function Declarations
        *********************************/
        public:
        
            /**
             * Default constructor
            **/
            vector3() : x(0), y(0), z(0);
            
            /**
             * Constructor which enters values that the users sends
            **/
            vector3(marb x, marb y, marb z) : x(x), y(y), z(z);
            
            /**
             * Inverts all the values of the vector
            **/
            vector3 invert()
            {
                x = -x;
                y = -y;
                z = -z;
            }
            
    }
}

#endif
