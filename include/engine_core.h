/*************************************************************
 * engine_core.h
 * -------------
 * Header file for the physics engine core. Defines
 * vectors and their functions.
 *
 * Last Revision: Oct. 21, 2014
 *
 * TO DO: - Continue following tutorial to fill this out.
 *************************************************************/

#ifndef ENGINE_CORE_INCLUDED
#define ENGINE_CORE_INCLUDED


#include <math.h>
#include "decimal_precision.h"
#include <iostream>

namespace marballs {

    extern marb sleepEpsilon; // Minimum value before a body is put to sleep.

    void SetSleepEpsilon(marb value); // Sets sleep epsilon value.

    marb GetSleepEpsilon(); // Returns sleep epsilon value.

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


//CLASS Quaternion - Holds a three degree of freedom orientation.
    class Quaternion
    {
    public:
        union {
            struct {
                //Holds the marb component of the quaternion.
                marb r;

                //Holds the first complex component of the quaternion
                marb i;

                //Holds the second complex component of the quaternion
                marb j;

                //Holds the third complex component of the quaternion
                marb k;
            };

            //Holds the quaternion data in array form.
            marb data[4];
        };

        // Default constructor
        Quaternion() : r(1), i(0), j(0), k(0) {}

        // Explicit constructor
        Quaternion(const marb r, const marb i, const marb j, const marb k)
            : r(r), i(i), j(j), k(k)
        {
        }

        //normalizes the quaternion to unit length, making it a valid orientation quaternion
        void Normalize()
        {
            marb d = r*r+i*i+j*j+k*k;

            //Check for zero length quaternion, and use the no-rotation quaternion in that case.
            if(d==0)
            {
                r=1;
                return;
            }
            d = ((marb)1.0)/marb_sqrt(d);
            r *= d;
            i *= d;
            j *= d;
            k *= d;
        }

        // Multiplies the quaternion by the given quaternion.
        void operator *=(const Quaternion &multiplier)
        {
            Quaternion q = *this;
            r = q.r*multiplier.r - q.i*multiplier.i -
            q.j*multiplier.j - q.k*multiplier.k;
            i = q.r*multiplier.i + q.i*multiplier.r +
            q.j*multiplier.k - q.k*multiplier.j;
            j = q.r*multiplier.j + q.j*multiplier.r +
            q.k*multiplier.i - q.i*multiplier.k;
            k = q.r*multiplier.k + q.k*multiplier.r +
            q.i*multiplier.j - q.j*multiplier.i;
        }

        void RotateByVector(const Vector3& vector)
        {
            Quaternion q(0, vector.x, vector.y, vector.z);
            (*this) *= q;
        }

        //adds the given vector to this, scaled by a given amount
        void AddScaledVector(const Vector3& vector, marb scale)
        {
            Quaternion q(0,
                         vector.x * scale,
                         vector.y * scale,
                         vector.z * scale);
            q *= *this;
            r += q.r * ((marb)0.5);
            i += q.i * ((marb)0.5);
            j += q.j * ((marb)0.5);
            k += q.k * ((marb)0.5);
        }
    };//End of quaternion class


    //CLASS Matrix3 - Holds inertia tensor, consisting of a 3x3 matrix.
    class Matrix3
    {
    public:
        marb data[9];//Holds tensor matix data in an array


        //Creates a new matrix.
        Matrix3()
        {
            data[0] = data[1] = data[2] = data[3] = data[4] = data[5] =
                data[6] = data[7] = data[8] = 0;
        }

        // Explicit constructor
        Matrix3(marb c0, marb c1, marb c2, marb c3, marb c4, marb c5,
            marb c6, marb c7, marb c8)
        {
            data[0] = c0; data[1] = c1; data[2] = c2;
            data[3] = c3; data[4] = c4; data[5] = c5;
            data[6] = c6; data[7] = c7; data[8] = c8;
        }

        Vector3 operator*(const Vector3 &vector) const
        {
            return Vector3(
                vector.x * data[0] + vector.y * data[1] + vector.z * data[2],
                vector.x * data[3] + vector.y * data[4] + vector.z * data[5],
                vector.x * data[6] + vector.y * data[7] + vector.z * data[8]
            );
        }

        // Returns a matrix multiplied by another matrix
        Matrix3 operator*(const Matrix3 &o) const
        {
            return Matrix3(
                data[0]*o.data[0] + data[1]*o.data[3] + data[2]*o.data[6],
                data[0]*o.data[1] + data[1]*o.data[4] + data[2]*o.data[7],
                data[0]*o.data[2] + data[1]*o.data[5] + data[2]*o.data[8],

                data[3]*o.data[0] + data[4]*o.data[3] + data[5]*o.data[6],
                data[3]*o.data[1] + data[4]*o.data[4] + data[5]*o.data[7],
                data[3]*o.data[2] + data[4]*o.data[5] + data[5]*o.data[8],

                data[6]*o.data[0] + data[7]*o.data[3] + data[8]*o.data[6],
                data[6]*o.data[1] + data[7]*o.data[4] + data[8]*o.data[7],
                data[6]*o.data[2] + data[7]*o.data[5] + data[8]*o.data[8]
                   );
        }

        // Sets the matrix to be the inverse of the given matrix. Parameter is the matrix to invert.

        void SetInverse(const Matrix3 &m)
        {
            marb t4 = m.data[0]*m.data[4];
            marb t6 = m.data[0]*m.data[5];
            marb t8 = m.data[1]*m.data[3];
            marb t10 = m.data[2]*m.data[3];
            marb t12 = m.data[1]*m.data[6];
            marb t14 = m.data[2]*m.data[6];

            // Calculate the determinant
            marb t16 = (t4*m.data[8] - t6*m.data[7] - t8*m.data[8]+
                        t10*m.data[7] + t12*m.data[5] - t14*m.data[4]);

            // Make sure the determinant is non-zero.
            if (t16 == (marb)0.0f) return;
            marb t17 = 1/t16;

            data[0] = (m.data[4]*m.data[8]-m.data[5]*m.data[7])*t17;
            data[1] = -(m.data[1]*m.data[8]-m.data[2]*m.data[7])*t17;
            data[2] = (m.data[1]*m.data[5]-m.data[2]*m.data[4])*t17;
            data[3] = -(m.data[3]*m.data[8]-m.data[5]*m.data[6])*t17;
            data[4] = (m.data[0]*m.data[8]-t14)*t17;
            data[5] = -(t6-t10)*t17;
            data[6] = (m.data[3]*m.data[7]-m.data[4]*m.data[6])*t17;
            data[7] = -(m.data[0]*m.data[7]-t12)*t17;
            data[8] = (t4-t8)*t17;
        }

        // Returns a new matrix containing the inverse of this matrix.
        Matrix3 Inverse() const
        {
            Matrix3 result;
            result.SetInverse(*this);
            return result;
        }

        // Inverts the matrix
        void invert()
        {
            SetInverse(*this);
        }

        // Sets matrix to be the transpose of the given matrix
        void SetTranspose(const Matrix3 &m)
        {
            data[0] = m.data[0];
            data[1] = m.data[3];
            data[2] = m.data[6];
            data[3] = m.data[1];
            data[4] = m.data[4];
            data[5] = m.data[7];
            data[6] = m.data[2];
            data[7] = m.data[5];
            data[8] = m.data[8];
        }

        // Returns a new matrix containing the transpose of this matrix.
        Matrix3 Transpose() const
        {
            Matrix3 result;
            result.SetTranspose(*this);
            return result;
        }

        // Sets this matrix to be the rotation matrix corresponding to the given quaternion.
        void SetOrientation(const Quaternion &q)
        {
            data[0] = 1 - (2*q.j*q.j + 2*q.k*q.k);
            data[1] = 2*q.i*q.j + 2*q.k*q.r;
            data[2] = 2*q.i*q.k - 2*q.j*q.r;
            data[3] = 2*q.i*q.j - 2*q.k*q.r;
            data[4] = 1 - (2*q.i*q.i  + 2*q.k*q.k);
            data[5] = 2*q.j*q.k + 2*q.i*q.r;
            data[6] = 2*q.i*q.k + 2*q.j*q.r;
            data[7] = 2*q.j*q.k - 2*q.i*q.r;
            data[8] = 1 - (2*q.i*q.i  + 2*q.j*q.j);
        }

        // Sets the value of the matrix from inertia tensor values.
        void SetInertiaTensorCoeffs(marb ix, marb iy, marb iz,
            marb ixy=0, marb ixz=0, marb iyz=0)
        {
            data[0] = ix;
            data[1] = data[3] = -ixy;
            data[2] = data[6] = -ixz;
            data[4] = iy;
            data[5] = data[7] = -iyz;
            data[8] = iz;
        }

        /*
         * Sets the value of the matrix as an inertia tensor of
         * a rectangular block aligned with the body's coordinate
         * system with the given axis half-sizes and mass.
         */
        void SetBlockInertiaTensor(const Vector3 &halfSizes, marb mass)
        {
            Vector3 squares = halfSizes.ComponentProduct(halfSizes);
            SetInertiaTensorCoeffs(0.3f*mass*(squares.y + squares.z),
                0.3f*mass*(squares.x + squares.z),
                0.3f*mass*(squares.x + squares.y));
        }

        // Transform the given vector by this matrix.
        Vector3 Transform(const Vector3 &vector) const
        {
            return (*this) * vector;
        }

        // Interpolates a couple of matrices.
        static Matrix3 LinearInterpolate(const Matrix3& a, const Matrix3& b, marb prop);
        
        void SetComponents(const Vector3 &compOne, const Vector3 &compTwo, const Vector3 &compThree){
            data[0] = compOne.x;
            data[1] = compTwo.x;
            data[2] = compThree.x;
            data[3] = compOne.y;
            data[4] = compTwo.y;
            data[5] = compThree.y;
            data[6] = compOne.z;
            data[7] = compTwo.z;
            data[8] = compThree.z;
        }

    };//End of Matrix3 class

    /*CLASS Matrix4 - Holds a transform matrix consisting of a rotation matrix and a position.
     * The matrix consists of 12 elements and it is assumed the remaining 4 are (0, 0, 0, 1) */
    class Matrix4
    {
    public:
        marb data[12];//Holds transform matrix data in an array

        //Transform the given vector by this matrix. Parameter is the vector to transform.

        Vector3 operator*(const Vector3 &vector) const
        {
            return Vector3(
                vector.x * data[0] +
                vector.y * data[1] +
                vector.z * data[2] + data[3],

                vector.x * data[4] +
                vector.y * data[5] +
                vector.z * data[6] + data[7],

                vector.x * data[8] +
                vector.y * data[9] +
                vector.z * data[10] + data[11]
                    );
        }

        // Returns a matrix multiplied by another matrix
        Matrix4 operator*(const Matrix4 &o) const
        {
            Matrix4 result;
            result.data[0] = (o.data[0]*data[0]) + (o.data[4]*data[1]) + (o.data[8]*data[2]);
            result.data[4] = (o.data[0]*data[4]) + (o.data[4]*data[5]) + (o.data[8]*data[6]);
            result.data[8] = (o.data[0]*data[8]) + (o.data[4]*data[9]) + (o.data[8]*data[10]);

            result.data[1] = (o.data[1]*data[0]) + (o.data[5]*data[1]) + (o.data[9]*data[2]);
            result.data[5] = (o.data[1]*data[4]) + (o.data[5]*data[5]) + (o.data[9]*data[6]);
            result.data[9] = (o.data[1]*data[8]) + (o.data[5]*data[9]) + (o.data[9]*data[10]);

            result.data[2] = (o.data[2]*data[0]) + (o.data[6]*data[1]) + (o.data[10]*data[2]);
            result.data[6] = (o.data[2]*data[4]) + (o.data[6]*data[5]) + (o.data[10]*data[6]);
            result.data[10] = (o.data[2]*data[8]) + (o.data[6]*data[9]) + (o.data[10]*data[10]);

            result.data[3] = (o.data[3]*data[0]) + (o.data[7]*data[1]) + (o.data[11]*data[2]) + data[3];
            result.data[7] = (o.data[3]*data[4]) + (o.data[7]*data[5]) + (o.data[11]*data[6]) + data[7];
            result.data[11] = (o.data[3]*data[8]) + (o.data[7]*data[9]) + (o.data[11]*data[10]) + data[11];

            return result;
        }

        // Returns determinant of the matrix
        marb GetDeterminant() const;

        // Sets the matrix to the inverse of the matrix
        void SetInverse(const Matrix4 &m);

        // Returns a new matrix containing the inverse of this matrix.
        Matrix4 Inverse() const
        {
            Matrix4 result;
            result.SetInverse(*this);
            return result;
        }

        // Inverts the matrix
        void Invert()
        {
            SetInverse(*this);
        }

        // Sets this matrix to be the rotation matrix corresponding to the given quaternion.
        void SetOrientationAndPos(const Quaternion &q, const Vector3 &pos)
        {
            data[0] = 1 - (2*q.j*q.j + 2*q.k*q.k);
            data[1] = 2*q.i*q.j + 2*q.k*q.r;
            data[2] = 2*q.i*q.k - 2*q.j*q.r;
            data[3] = pos.x;

            data[4] = 2*q.i*q.j - 2*q.k*q.r;
            data[5] = 1 - (2*q.i*q.i  + 2*q.k*q.k);
            data[6] = 2*q.j*q.k + 2*q.i*q.r;
            data[7] = pos.y;

            data[8] = 2*q.i*q.k + 2*q.j*q.r;
            data[9] = 2*q.j*q.k - 2*q.i*q.r;
            data[10] = 1 - (2*q.i*q.i  + 2*q.j*q.j);
            data[11] = pos.z;
        }

        Vector3 Transform(const Vector3 &vector) const
        {
            return (*this) * vector;
        }

        // Transform the given vector by the transformational inverse of this matrix.
        Vector3 TransformInverse(const Vector3 &vector) const
        {
            Vector3 tmp = vector;
            tmp.x -= data[3];
            tmp.y -= data[7];
            tmp.z -= data[11];
            return Vector3(
                           tmp.x * data[0] +
                           tmp.y * data[4] +
                           tmp.z * data[8],

                           tmp.x * data[1] +
                           tmp.y * data[5] +
                           tmp.z * data[9],

                           tmp.x * data[2] +
                           tmp.y * data[6] +
                           tmp.z * data[10]
                           );
        }

        // Transform the given direction vector by this matrix.
        Vector3 TransformDirection(const Vector3 &vector) const
        {
            return Vector3(
                           vector.x * data[0] +
                           vector.y * data[1] +
                           vector.z * data[2],

                           vector.x * data[4] +
                           vector.y * data[5] +
                           vector.z * data[6],

                           vector.x * data[8] +
                           vector.y * data[9] +
                           vector.z * data[10]
                           );
        }

        // Transform the given direction vector by the transformational inverse of this matrix.
        Vector3 TransformInverseDirection(const Vector3 &vector) const
        {
            return Vector3(
                           vector.x * data[0] +
                           vector.y * data[4] +
                           vector.z * data[8],

                           vector.x * data[1] +
                           vector.y * data[5] +
                           vector.z * data[9],

                           vector.x * data[2] +
                           vector.y * data[6] +
                           vector.z * data[10]
                           );
        }
        //Fills the given array with this transform matrix, so it is usable as an open-gl transform matrix.
        void fillGLArray(float array[16]) const
        {
            array[0] = (float)data[0];
            array[1] = (float)data[4];
            array[2] = (float)data[8];
            array[3] = (float)0;

            array[4] = (float)data[1];
            array[5] = (float)data[5];
            array[6] = (float)data[9];
            array[7] = (float)0;

            array[8] = (float)data[2];
            array[9] = (float)data[6];
            array[10] = (float)data[10];
            array[11] = (float)0;

            array[12] = (float)data[3];
            array[13] = (float)data[7];
            array[14] = (float)data[11];
            array[15] = (float)1;
        }
    }; //End of Matrix4 class

} // End of namespace.

#endif
