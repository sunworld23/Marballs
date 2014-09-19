/*************************************************************
 * engine_core.h
 * -------------
 * Header file for the physics engine core. Defines 
 * vectors and their functions.
 *
 * Last Revision: Sept. 19, 2014
 *
 * TO DO: - Continue following tutorial to fill this out.
 *		  - This bullet included for to do list formatting.
 *************************************************************/

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
    /* CLASS vector3 - Holds the values for a 3D vector */
    class vector3
    {
        /*********************************
         *     Variable Declarations
        *********************************/
        public:
        
            marb x; // Holds the x-axis value
            marb y; // Holds the y-axis value
            marb z; // Holds the z-access value
        
        private:

            marb pad; // From book: Padding variable for memory performance, optional.
        
        /*********************************
         *     Function Declarations
        *********************************/
        public:
        
            // vector3 - Default Constructor
            vector3() : x(0), y(0), z(0);
            
            // vector3 - Constructor that initializes x, y, and z.
            vector3(marb x, marb y, marb z) : x(x), y(y), z(z);
            
            // invert - Inverts a vector3's x, y, and z values.
            vector3 invert()
            {
                x = -x;
                y = -y;
                z = -z;
            }
            
    }
}

#endif
