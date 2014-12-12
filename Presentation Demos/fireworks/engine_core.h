/*************************************************************
 * engine_core.h
 * -------------
 * Header file for the physics engine core. Defines
 * vectors and their functions.
 *
 * Last Revision: Sept. 28, 2014
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
#include <iostream>

namespace marballs
{
    /* CLASS Vector3 - Holds the values for a 3D vector */
    class Vector3
    {
        /*********************************
         *     Variable Declarations
        *********************************/
        public:

            marb x; // Holds the x-axis value
            marb y; // Holds the y-axis value
            marb z; // Holds the z-access value

            const static Vector3 GRAVITY; // Holds constant gravity value.

        private:

            marb pad; // From book: Padding variable for memory performance, optional.

        /*********************************
         *     Function Declarations
        *********************************/
        public:

            // Vector3 - Default Constructor
            Vector3() : x(0), y(0), z(0) {}

            // Vector3 - Constructor that initializes x, y, and z.
            Vector3(marb x, marb y, marb z) : x(x), y(y), z(z) {}

            // Invert - Inverts a Vector3's x, y, and z values.
            void Invert() {
                x = -x;
                y = -y;
                z = -z;
            }

			// Magnitude - Returns the magnitude (length) of this vector.
			marb Magnitude() const {
				return marb_sqrt(x*x+y*y+z*z);
			}

			// SquareMagnitude - Returns the squared magnitude of this vector.
			marb SquareMagnitude() const {
				return x*x+y*y+z*z;
			}

			// Normalize - Changes vector's magnitude to 1 while maintaining direction.
			void Normalize() {
				marb length = Magnitude();
				if (length > 0) {
					(*this)*=((marb)1)/length; // Multiplies vector by reciprocal of its length.
				}
			}

			// Clear - Zeros all components of vector.
			void Clear() { x = y = z = 0; }

			// *= Operater overload - multiplies vector by a scalar value
			void operator*=(const marb scalar) {
                x *= scalar;
                y *= scalar;
                z *= scalar;
			}

			/* NOTE: Previously avoided inclusion of overloaded * because pointers, use this carefully. */
			// * Operator Overload - returns a copy of this Vector3 multiplied by a scalar
			Vector3 operator*(const marb scalar) {
                return Vector3(x*scalar, y*scalar, z*scalar);
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
			void addScaledVector(const Vector3& vector, marb scalar) {
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
			marb dotProduct(const Vector3 &vector) const {
				return x*vector.x + y*vector.y + z*vector.z; // Returns a number, not a vector!
			}

			// scalarProduct - See: dotProduct
			// NOTE: Leaving it in as a deprecated function with a suggestion to use the other.
			// 		 This way, people following the guide may use this without confusion, but are
			//		 pointed in the right direction.
			marb scalarProduct(const Vector3 &vector) const {
				std::cout << "[DEBUG] (engine_core.h) Use dotProduct() instead!" << std::endl;
				return dotProduct(vector);
			}

			// crossProduct - Calculates and returns the cross product of this vector and the given one.
			// Used for calculating normal vectors, and determining if vectors are parallel.
			// NOTE: With vectors a and b, a x b = -b x a
			Vector3 crossProduct(const Vector3 &vector) const {
				return Vector3(y * vector.z - z * vector.y,
							   z * vector.x - x * vector.z,
							   x * vector.y - y * vector.x);
			}

			// vectorProduct - See: crossProduct
			// NOTE: Leaving it in as a deprecated function with a suggestion to use the other.
			// 		 This way, people following the guide may use this without confusion, but are
			//		 pointed in the right direction.
			Vector3 vectorProduct(const Vector3 &vector) const {
				std::cout << "[DEBUG] (engine_core.h) Use crossProduct() instead!" << std::endl;
				return crossProduct(vector);
			}

			// overload << operator - To help with testing and getting values of Vector3
            //                      - must be friend function since it takes user defined argument
			friend std::ostream &operator<<(std::ostream &os, const Vector3 v) {
                os << "<" << v.x << ", " << v.y << ", " << v.z << ">";
                return os;
			}

    }; // Vector3 class end
} // marballs namespace end

#endif  //ENGINE_CORE_INCLUDED
