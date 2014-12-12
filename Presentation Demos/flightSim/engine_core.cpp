/*************************************************************
 * engine_core.cpp
 * ---------------
 * Source code for physics engine core.
 *
 * Last Revision: Sept. 28, 2014
 *
 * TO DO: - Continue following tutorial to fill this out.
 *************************************************************/

 #include "engine_core.h"

 using namespace marballs;

 const Vector3 Vector3::GRAVITY = Vector3(0, -9.81, 0);
 const Vector3 Vector3::UP = Vector3(0, 1, 0);

 marb marballs::sleepEpsilon = ((marb)0.3);

 void marballs::SetSleepEpsilon(marb value)
{
    marballs::sleepEpsilon = value;
}

marb marballs::GetSleepEpsilon()
{
    return marballs::sleepEpsilon;
}

marb Matrix4::GetDeterminant() const
{
    return -data[8]*data[5]*data[2]+
    data[4]*data[9]*data[2]+
    data[8]*data[1]*data[6]-
    data[0]*data[9]*data[6]-
    data[4]*data[1]*data[10]+
    data[0]*data[5]*data[10];
}

void Matrix4::SetInverse(const Matrix4 &m)
{
    // Make sure the determinant is non-zero.
    marb det = GetDeterminant();
    if (det == 0) return;
    det = ((marb)1.0)/det;

    data[0] = (-m.data[9]*m.data[6]+m.data[5]*m.data[10])*det;
    data[4] = (m.data[8]*m.data[6]-m.data[4]*m.data[10])*det;
    data[8] = (-m.data[8]*m.data[5]+m.data[4]*m.data[9])*det;

    data[1] = (m.data[9]*m.data[2]-m.data[1]*m.data[10])*det;
    data[5] = (-m.data[8]*m.data[2]+m.data[0]*m.data[10])*det;
    data[9] = (m.data[8]*m.data[1]-m.data[0]*m.data[9])*det;

    data[2] = (-m.data[5]*m.data[2]+m.data[1]*m.data[6])*det;
    data[6] = (+m.data[4]*m.data[2]-m.data[0]*m.data[6])*det;
    data[10] = (-m.data[4]*m.data[1]+m.data[0]*m.data[5])*det;

    data[3] = (m.data[9]*m.data[6]*m.data[3]
               -m.data[5]*m.data[10]*m.data[3]
               -m.data[9]*m.data[2]*m.data[7]
               +m.data[1]*m.data[10]*m.data[7]
               +m.data[5]*m.data[2]*m.data[11]
               -m.data[1]*m.data[6]*m.data[11])*det;
    data[7] = (-m.data[8]*m.data[6]*m.data[3]
               +m.data[4]*m.data[10]*m.data[3]
               +m.data[8]*m.data[2]*m.data[7]
               -m.data[0]*m.data[10]*m.data[7]
               -m.data[4]*m.data[2]*m.data[11]
               +m.data[0]*m.data[6]*m.data[11])*det;
    data[11] =(m.data[8]*m.data[5]*m.data[3]
               -m.data[4]*m.data[9]*m.data[3]
               -m.data[8]*m.data[1]*m.data[7]
               +m.data[0]*m.data[9]*m.data[7]
               +m.data[4]*m.data[1]*m.data[11]
               -m.data[0]*m.data[5]*m.data[11])*det;
}

Matrix3 Matrix3::LinearInterpolate(const Matrix3& a, const Matrix3& b, marb prop)
{
    Matrix3 result;
    for (unsigned i = 0; i < 9; i++) {
        result.data[i] = a.data[i] * (1-prop) + b.data[i] * prop;
    }
    return result;
}
