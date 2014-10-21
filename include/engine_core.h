/*************************************************************
 * engine_core.h
 * -------------
 * Header file for the physics engine core. Defines
 * vectors and their functions.
 *
 * Last Revision: Oct. 20, 2014
 *
 * TO DO: - Continue following tutorial to fill this out.
 *************************************************************/

#ifndef ENGINE_CORE_INCLUDED
#define ENGINE_CORE_INCLUDED


#include <math.h>
#include "decimal_precision.h"
#include <iostream>

namespace marballs {

    //extern marb sleepEpsilon; // Minimum value before a body is put to sleep.

    //void SetSleepEpsilon(marb value); // Sets sleep epsilon value.

    //marb GetSleepEpsilon(); // Returns sleep epsilon value.

    /* CLASS Vector3 - Holds the values for a 3D vector */
    class Vector3 {
        /*********************************
         *     Variable Declarations
        *********************************/
        public:

            marb x; // Holds the x-axis value
            marb y; // Holds the y-axis value
            marb z; // Holds the z-access value

            const static Vector3 GRAVITY;
            //const static Vector3 HIGH_GRAVITY;
            const static Vector3 UP;
            //const static Vector3 RIGHT;
            //const static Vector3 OUT_OF_SCREEN;
            //const static Vector3 X;
            //const static Vector3 Y;
            //const static Vector3 Z;

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

            // Magnitude - Returns the magnitude (length) of this vector.
			marb Magnitude() const {
				return marb_sqrt(x*x+y*y+z*z);
			}

			// SquareMagnitude - Returns the squared magnitude of this vector.
			marb SquareMagnitude() const {
				return x*x+y*y+z*z;
			}

			// Clear - Zeros all components of vector.
			void Clear() { x = y = z = 0; }


            // AddScaledVector - Adds a given vector to this, scaled by a scalar.
			void AddScaledVector(const Vector3& vector, marb scalar) {
				x += vector.x * scalar;
				y += vector.y * scalar;
				z += vector.z * scalar;
			}

			// ComponentProduct - Straight multiplies the components of this vector and the given one and returns the result.
			Vector3 ComponentProduct(const Vector3 &vector) const {
				return Vector3(x * vector.x, y * vector.y, z * vector.z);
			}

			// ComponentProductUpdate - Performs a component product and sets this vector to the result.
			void ComponentProductUpdate(const Vector3 &vector) {
				x *= vector.x;
				y *= vector.y;
				z *= vector.z;
			}

			// DotProduct - Calculates and returns the dot product of this vector and the given one.
			// Used for determining if vectors are perpendicular, and what the angle is between them.
			marb DotProduct(const Vector3 &vector) const {
				return x*vector.x + y*vector.y + z*vector.z; // Returns a number, not a vector!
			}

			// ScalarProduct - See: DotProduct
			// NOTE: Leaving it in as a deprecated function with a suggestion to use the other.
			// 		 This way, people following the guide may use this without confusion, but are
			//		 pointed in the right direction.
			marb ScalarProduct(const Vector3 &vector) const {
				std::cout << "[DEBUG] (engine_core.h) Use dotProduct() instead!" << std::endl;
				return DotProduct(vector);
			}

            // CrossProduct - Calculates and returns the cross product of this vector and the given one.
			// Used for calculating normal vectors, and determining if vectors are parallel.
			// NOTE: With vectors a and b, a x b = -b x a
			Vector3 CrossProduct(const Vector3 &vector) const {
				return Vector3(y * vector.z - z * vector.y,
							   z * vector.x - x * vector.z,
							   x * vector.y - y * vector.x);
			}

			// VectorProduct - See: CrossProduct
			// NOTE: Leaving it in as a deprecated function with a suggestion to use the other.
			// 		 This way, people following the guide may use this without confusion, but are
			//		 pointed in the right direction.
			Vector3 VectorProduct(const Vector3 &vector) const {
				std::cout << "[DEBUG] (engine_core.h) Use crossProduct() instead!" << std::endl;
				return CrossProduct(vector);
			}

            // Trim - Limits vector to given size.
            void Trim(marb size) {
                if (SquareMagnitude() > size*size) {
                    Normalize();
                    x *= size;
                    y *= size;
                    z *= size;
                }
            }

            // Normalize - Changes vector's magnitude to 1 while maintaining direction.
            void Normalize() {
                marb length = Magnitude();
                if (length > 0) {
                    (*this)*=((marb)1)/length; // Multiplies vector by reciprocal of its length.
                }
            }

            // Unit - Returns normalized version of vector.
            Vector3 Unit() const {
                Vector3 result = *this;
                result.Normalize();
                return result;
            }

            // Invert - Inverts a Vector3's x, y, and z values.
            void Invert() {
                x = -x;
                y = -y;
                z = -z;
            }

           /**********************
            * Operator Overloads
            **********************/

             /*marb operator[](unsigned i) const {
                if (i == 0) return x;
                if (i == 1) return y;
                return z;
            }

            marb& operator[](unsigned i) {
                if (i == 0) return x;
                if (i == 1) return y;
                return z;
            }*/

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

            // *= Operater overload - multiplies vector by a scalar value
            void operator*=(const marb scalar) {
                x *= scalar;
                y *= scalar;
                z *= scalar;
            }

            // * Operator Overload - returns a copy of this Vector3 multiplied by a scalar
            Vector3 operator*(const marb scalar) {
                return Vector3(x*scalar, y*scalar, z*scalar);
            }

            // %= Operator Overload - Updates this vector to be its cross product with the given vector.
            void operator %=(const Vector3 &vector) {
                *this = CrossProduct(vector);
            }

            // % Operator Overload - Returns the cross product between this vector and the given one.
            Vector3 operator%(const Vector3 &vector) const {
                return Vector3(y*vector.z-z*vector.y,
                               z*vector.x-x*vector.z,
                               x*vector.y-y*vector.x);
            }

            // * Operator Overload - Returns dot product between this vector and the given one.
            marb operator *(const Vector3 &vector) const {
                return x*vector.x + y*vector.y + z*vector.z;
            }

            // == Operator Overload - True if vectors are same.
            bool operator==(const Vector3& other) const {
                return x == other.x && y == other.y && z == other.z;
            }

            // != Operator Overload - True if vectors not same.
            bool operator!=(const Vector3& other) const {
                return !(*this == other);
            }

            // < Operator Overload - True if every component of this vector is smaller than the given's.
            bool operator<(const Vector3& other) const {
                return x < other.x && y < other.y && z < other.z;
            }

            // > Operator Overload - True if every component of this vector is larger than the given's.
            bool operator>(const Vector3& other) const {
                return x > other.x && y > other.y && z > other.z;
            }

            // <= Operator Overload - Similar to <.
            bool operator<=(const Vector3& other) const {
                return x <= other.x && y <= other.y && z <= other.z;
            }

            // >= Operator Overload - Similar to >.
            bool operator>=(const Vector3& other) const {
                return x >= other.x && y >= other.y && z >= other.z;
            }

    }; // End of Vector3 class.

} // End of namespace.

#endif
