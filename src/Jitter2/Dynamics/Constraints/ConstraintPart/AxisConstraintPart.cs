using Jitter2.LinearMath;
using System;

namespace Jitter2.Dynamics
{
    /// <summary>
    /// Constraint that constrains motion along 1 axis
    /// </summary>
    internal struct AxisConstraintPart
    {
        public JVector Axis;
        /// <summary>
        /// Accumulated Impulse
        /// </summary>
        public float AccImpulse;
        public float EffectiveMass;

        /// <summary>
        /// RelativePos1  Cross Axis
        /// </summary>
        public JVector RelP1CroAxis;

        /// <summary>
        /// RelativePos2  Cross Axis
        /// </summary>
        public JVector RelP2CroAxis;

        /// <summary>
        /// RelativePos1  Cross Axis InverseInertiaWorld
        /// </summary>
        public JVector RelP1CroAxisInvI;

        /// <summary>
        /// RelativePos2  Cross Axis InverseInertiaWorld
        /// </summary>
        public JVector RelP2CroAxisInvI;

        public void CalculateConstraintProperties(ref RigidBodyData b1, ref RigidBodyData b2, JVector relP1, JVector relP2, JVector axisWS, float bias = 0)
        {
            float invEffectiveMass = CalculateInverseEffectiveMass(ref b1, ref b2, relP1, relP2, axisWS);
            if (invEffectiveMass == 0.0f)
            {
                Deactivate();
            }
            else
            {
                EffectiveMass = 1.0f / invEffectiveMass;
            }
        }

        private float CalculateInverseEffectiveMass(ref RigidBodyData b1, ref RigidBodyData b2, JVector relP1, JVector relP2, JVector axisWS)
        {
            if (b1.MotionType != BodyMotionType.Static)
            {
                RelP1CroAxis = JVector.Cross(relP1, axisWS);
            }
            else
            {
                RelP1CroAxis = JVector.NaN;
            }

            if (b2.MotionType != BodyMotionType.Static)
            {
                RelP2CroAxis = JVector.Cross(relP2, axisWS);
            }
            else
            {
                RelP2CroAxis = JVector.NaN;
            }

            // Calculate inverse effective mass: K = J M^-1 J^T
            float inv_effective_mass = 0;
            if (b1.MotionType == BodyMotionType.Dynamic)
            {
                JVector.Transform(RelP1CroAxis, b1.InverseInertiaWorld, out RelP1CroAxisInvI);
                inv_effective_mass = b1.InverseMass + JVector.Dot(RelP1CroAxisInvI, RelP1CroAxis);
            }
            else
            {
                RelP1CroAxisInvI = JVector.NaN;
                inv_effective_mass = 0.0f;
            }

            if (b2.MotionType == BodyMotionType.Dynamic)
            {
                JVector.Transform(RelP2CroAxis, b2.InverseInertiaWorld, out RelP2CroAxisInvI);
                inv_effective_mass = b2.InverseMass + JVector.Dot(RelP2CroAxisInvI, RelP2CroAxis);
            }
            else
            {
                RelP2CroAxisInvI = JVector.NaN;
            }

            return inv_effective_mass;
        }

        /// Deactivate this constraint
        public void Deactivate()
        {
            EffectiveMass = 0.0f;
            AccImpulse = 0.0f;
        }

        /// Check if constraint is active
        public bool IsActive()
        {
            return EffectiveMass != 0.0f;
        }

        public void WarmStart(ref RigidBodyData b1, ref RigidBodyData b2, JVector axisWS, float warmStartImpulseRatio)
        {
            AccImpulse *= warmStartImpulseRatio;

            // Apply impulse if delta is not zero
            if (AccImpulse != 0.0f)
            {
                // Calculate velocity change due to constraint
                //
                // Impulse:
                // P = J^T lambda
                //
                // Euler velocity integration:
                // v' = v + M^-1 P
                if (b1.MotionType == BodyMotionType.Dynamic)
                {
                    b1.Velocity -= (AccImpulse * b1.InverseMass) * axisWS;
                    b1.AngularVelocity -= AccImpulse * RelP1CroAxisInvI;
                }

                if (b2.MotionType == BodyMotionType.Dynamic)
                {
                    b2.Velocity += (AccImpulse * b2.InverseMass) * axisWS;
                    b2.AngularVelocity += AccImpulse * RelP2CroAxisInvI;
                }
            }

        }

        public float SolveVelocityConstraintGetAccImpulse(ref RigidBodyData b1, ref RigidBodyData b2, JVector axisWS)
        {
            // Calculate jacobian multiplied by linear velocity
            float jv = 0;
            if (b1.MotionType != BodyMotionType.Static && b2.MotionType != BodyMotionType.Static)
            {
                jv = JVector.Dot(axisWS, b1.Velocity - b2.Velocity);
            }
            else if (b1.MotionType != BodyMotionType.Static)
            {
                jv = JVector.Dot(axisWS, b1.Velocity);
            }
            else if (b2.MotionType != BodyMotionType.Static)
            {
                jv = JVector.Dot(axisWS, -b2.Velocity);
            }

            // Calculate jacobian multiplied by angular velocity
            if (b1.MotionType != BodyMotionType.Static)
            {
                jv += JVector.Dot(RelP1CroAxis, b1.AngularVelocity);
            }
            if (b2.MotionType != BodyMotionType.Static)
            {
                jv -= JVector.Dot(RelP2CroAxis, b2.AngularVelocity);
            }

            // Lagrange multiplier is:
            //
            // lambda = -K^-1 (J v + b)
            float lambda = EffectiveMass * jv;

            // Return the total accumulated lambda
            return AccImpulse + lambda;
        }

        public bool SolveVelocityConstraintApplyLambda(ref RigidBodyData b1, ref RigidBodyData b2, JVector axisWS, float lambda)
        {
            float delta_lambda = lambda - AccImpulse; // Calculate change in lambda
            AccImpulse = lambda; // Store accumulated impulse

            // Apply impulse if delta is not zero
            if (delta_lambda != 0.0f)
            {
                // Calculate velocity change due to constraint
                //
                // Impulse:
                // P = J^T lambda
                //
                // Euler velocity integration:
                // v' = v + M^-1 P
                if (b1.MotionType == BodyMotionType.Dynamic)
                {
                    b1.Velocity -= delta_lambda * b1.InverseMass * axisWS;
                    b1.AngularVelocity -= delta_lambda * RelP1CroAxisInvI;
                }

                if (b2.MotionType == BodyMotionType.Dynamic)
                {
                    b2.Velocity += delta_lambda * b2.InverseMass * axisWS;
                    b2.AngularVelocity += delta_lambda * RelP2CroAxisInvI;
                }
                return true;
            }

            return false;
        }

        public bool SolveVelocityConstraint(ref RigidBodyData b1, ref RigidBodyData b2, JVector axisWS, float minLambda, float maxLambda)
        {
            float total_lambda = SolveVelocityConstraintGetAccImpulse(ref b1, ref b2, axisWS);

            // Clamp impulse to specified range
            total_lambda = Math.Clamp(total_lambda, minLambda, maxLambda);

            return SolveVelocityConstraintApplyLambda(ref b1, ref b2, axisWS, total_lambda);
        }

        public bool SolvePositionConstraintWithMassOverride(ref RigidBodyData b1, ref RigidBodyData b2, JVector axisWS, float separation, float inBaumgarte)
        {
            if (separation != 0)
            {
                // Calculate lagrange multiplier (lambda) for Baumgarte stabilization:
                //
                // lambda = -K^-1 * beta / dt * C
                //
                // We should divide by inDeltaTime, but we should multiply by inDeltaTime in the Euler step below so they're cancelled out
                float lambda = -EffectiveMass * inBaumgarte * separation;

                // Directly integrate velocity change for one time step
                //
                // Euler velocity integration:
                // dv = M^-1 P
                //
                // Impulse:
                // P = J^T lambda
                //
                // Euler position integration:
                // x' = x + dv * dt
                //
                // Note we don't accumulate velocities for the stabilization. This is using the approach described in 'Modeling and
                // Solving Constraints' by Erin Catto presented at GDC 2007. On slide 78 it is suggested to split up the Baumgarte
                // stabilization for positional drift so that it does not actually add to the momentum. We combine an Euler velocity
                // integrate + a position integrate and then discard the velocity change.
                if (b1.MotionType == BodyMotionType.Dynamic)
                {
                    b1.Position -= lambda * b1.InverseMass * axisWS;

                    //ioBody1.SubRotationStep(lambda * Vec3::sLoadFloat3Unsafe(mInvI1_R1PlusUxAxis));
                    var deltaAngle = lambda * RelP1CroAxisInvI;
                    var deltaAngleLenght = deltaAngle.Length();
                    if (deltaAngleLenght > 0.000001)
                    {
                        var curRotation = b1.Orientation;
                        curRotation = curRotation * JQuaternion.AngleAxis(-deltaAngleLenght, 1f / deltaAngleLenght * deltaAngle);
                        curRotation.Normalize();
                        b1.Orientation = curRotation;
                    }
                }

                if (b2.MotionType == BodyMotionType.Dynamic)
                {
                    b2.Position += lambda * b2.InverseMass * axisWS;

                    //ioBody2.AddRotationStep(lambda * Vec3::sLoadFloat3Unsafe(mInvI2_R2xAxis));
                    var deltaAngle = lambda * RelP2CroAxisInvI;
                    var deltaAngleLenght = deltaAngle.Length();
                    if (deltaAngleLenght > 0.000001)
                    {
                        var curRotation = b2.Orientation;
                        curRotation = curRotation * JQuaternion.AngleAxis(deltaAngleLenght, 1f / deltaAngleLenght * deltaAngle);
                        curRotation.Normalize();
                        b2.Orientation = curRotation;
                    }

                }

                return true;
            }

            return false;
        }

    }
}
