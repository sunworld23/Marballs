/*************************************************************
 * engine_core.h
 * -------------
 * Header file for the physics engine core. Defines
 * vectors and their functions.
 *
 * Last Revision: Sept. 22, 2014
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
            vector3 invert() {
                x = -x;
                y = -y;
                z = -z;
            }

			// magnitude - Returns the magnitude (length) of this vector.
			real magnitude() const {
				return marb_sqrt(x*x+y*y+z*z);
			}
			
			// squareMagnitude - Returns the squared magnitude of this vector.
			real squareMagnitude() const {
				return x*x+y*y+z*z;
			}
			
			// normalize - Changes vector's magnitude to 1 while maintaining direction.
			void normalize() {
				real length = magnitude();
				if (length > 0) {
					(*this)*=((real)1)/length; // Multiplies vector by reciprocal of its length.
				}
			}
			
			// += Operator Overload - Adds vector components to this vector.
			void operator+=(const Vector3& v) {
				x += v.x;
				y += v.y;
				z += v.z;
			}
			
			// + Operator Overload - Returns a vector whose components are the sum of given vectors.
			Vector3 operator+(const Vector3& v) const {
				return Vector3(x+v.x, y+v.y, z+v.z);
			}
			
			// -= Operator Overload - Subtracts vector components from this vector.
			void operator-=(const Vector3& v) {
				x -= v.x;
				y -= v.y;
				z -= v.z;
			}
			
			// - Operator Overload - Returns a vector whose components are the difference of given vectors.
			Vector3 operator-(const Vector3& v) const {
				return Vector3(x-v.x, y-v.y, z-v.z);
			}
			
			// addScaledVector - Adds a given vector to this, scaled by a scalar.
			void addScaledVector(const vector3& vector, real scalar) {
				x += vector.x * scalar;
				y += vector.y * scalar;
				z += vector.z * scalar;
			}
			
			// componentProduct - Straight multiplies the components of this vector and the given one and returns the result.
			Vector3 componentProduct(const Vector3 &vector) const {
				return Vector3(x * vector.x, y * vector.y, z * vector.z);
			}
			
			// componentProductUpdate - Performs a component product and sets this vector to the result.
			void componentProductUpdate(const Vector3 &vector) {
				x *= vector.x;
				y *= vector.y;
				z *= vector.z;
			}
			
			// dotProduct - Calculates and returns the dot product of this vector and the given one.
			// Used for determining if vectors are perpendicular, and what the angle is between them.
			real dotProduct(const Vector3 &vector) const {
				return x*vector.x + y*vector.y + z*vector.z; // Returns a number, not a vector!
			}
			
			// scalarProduct - See: dotProduct
			real scalarProduct(const Vector3 &vector) const {
				return dotProduct(vector);
			}
			
			/* NOTE: Avoiding inclusion of overloaded * because * is a confusing operator as is because pointers. */
			
			// crossProduct - Calculates and returns the cross product of this vector and the given one.
			// Used for calculating normal vectors, and determining if vectors are parallel.
			// NOTE: With vectors a and b, a x b = -b x a
			Vector3 crossProduct(const Vector3 &vector) const {
				return Vector3(y * vector.z - z * vector.y,
							   z * vector.x - x * vector.z,
							   x * vector.y - y * vector.x);
			}
			
			// vectorProduct - See: crossProduct
			Vector3 vectorProduct(const Vector3 &vector) const {
				return crossProduct(vector);
			}
    }
}

#endif  //ENGINE_CORE_INCLUDED
