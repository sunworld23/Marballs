#ifndef FGEN_INCLUDED
#define FGEN_INCLUDED

namespace cyclone
{
    /**
    * A force generator that applies an aerodynamic force.
    */
    class Aero : public ForceGenerator
    {
        /**
        * Holds the aerodynamic tensor for the surface in body space.
        */
        Matrix3 tensor;

        /**
        * Holds the relative position of the aerodynamic surface
        * in body coordinates.
        */
        Vector3 position;

        /**
        * Holds a pointer to a vector containing the wind speed of
        * the environment. This is easier than managing a separate
        * wind speed vector per generator and having to update
        * it manually as the wind changes.
        */
        const Vector3* windspeed;

    public:
        /**
        * Creates a new aerodynamic force generator with the
        * given properties.
        */
        Aero(const Matrix3 &tensor, const Vector3 &position,
            const Vector3 *windspeed);

        /**
        * Applies the force to the given rigid body.
        */
        virtual void updateForce(RigidBody *body, real duration);
    };

    /**
    * A force generator with a control aerodynamic surface. This
    * requires three inertia tensors, for the two extremes and the ’resting’
    * position of the control surface.
    * The latter tensor is the one inherited from the base class;
    * the two extremes are defined in this class.
    */
    class AeroControl : public Aero
    {
        /**
        * The aerodynamic tensor for the surface, when the control is at
        * its maximum value.
        */
        Matrix3 maxTensor;
        /**
        * The aerodynamic tensor for the surface, when the control is at
        * its minimum value.
        */
        Matrix3 minTensor;
        /**
        * The current position of the control for this surface. This
        * should range between -1 (in which case the minTensor value is
        * used) through 0 (where the base-class tensor value is used)
        * to +1 (where the maxTensor value is used).
        */
        real controlSetting;

    private:
        /**
        * Calculates the final aerodynamic tensor for the current
        * control setting.
        */
        Matrix3 getTensor();

    public:
        /**
        * Creates a new aerodynamic control surface with the given
        * properties.
        */
        AeroControl(const Matrix3 &base, const Matrix3 &min,
        const Matrix3 &max, const Vector3 &position,
        const Vector3 *windspeed);

        /**
        * Sets the control position of this control. This
        * should range between -1 (in which case the minTensor value is
        * used) through 0 (where the base-class tensor value is used)
        * to +1 (where the maxTensor value is used). Values outside that
        * range give undefined results.
        */
        void setControl(real value);

        /**
        * Applies the force to the given rigid body.
        */
        virtual void updateForce(RigidBody *body, real duration);

        };

}

#endif // FGEN_INCLUDED
