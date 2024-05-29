/*
 * Copyright (c) Thorben Linneweber and others
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

using System;
using System.Collections.Generic;
using Jitter2.LinearMath;
using Jitter2.UnmanagedMemory;

namespace Jitter2.Dynamics;

/// <summary>
/// Holds four <see cref="Contact"/> structs. The <see cref="ContactData.UsageMask"/>
/// indicates which contacts are actually in use. Every shape-to-shape collision in Jitter is managed
/// by one of these structs.
/// </summary>
public struct ContactData
{
    /// Maximum allowed distance between old and new contact point to preserve contact forces for warm start (units: meter^2)
	private const float ContactPointPreserveLambdaMaxDistSq = 0.0001f; ///< 1 cm

#pragma warning disable CS0649
    // Accessed in unsafe code.
    internal int _internal;
#pragma warning restore CS0649

    public int UsageMask;

    public JHandle<RigidBodyData> Body1;
    public JHandle<RigidBodyData> Body2;

    public ArbiterKey Key;

    private float Friction;
    private float Restitution;

    public bool IsSpeculative;

    public Contact Contact0;
    public Contact Contact1;
    public Contact Contact2;
    public Contact Contact3;
    //public List<Contact> Contacts;
    public JVector NormalWS;


    public void WarmStartVelocityConstraints(float dt)
    {
        if ((UsageMask & 0b0001) != 0) Contact0.WarmStartVelocityConstraints(ref Body1.Data, ref Body2.Data, NormalWS, dt, IsSpeculative);
        if ((UsageMask & 0b0010) != 0) Contact1.WarmStartVelocityConstraints(ref Body1.Data, ref Body2.Data, NormalWS, dt, IsSpeculative);
        if ((UsageMask & 0b0100) != 0) Contact2.WarmStartVelocityConstraints(ref Body1.Data, ref Body2.Data, NormalWS, dt, IsSpeculative);
        if ((UsageMask & 0b1000) != 0) Contact3.WarmStartVelocityConstraints(ref Body1.Data, ref Body2.Data, NormalWS, dt, IsSpeculative);
    }

    public void SolveVelocityConstraints()
    {
        // First apply all friction constraints (non-penetration is more important than friction)
        if ((UsageMask & 0b0001) != 0) Contact0.SolveVelocityFrictionConstraints(ref Body1.Data, ref Body2.Data, NormalWS, Friction);
        if ((UsageMask & 0b0010) != 0) Contact1.SolveVelocityFrictionConstraints(ref Body1.Data, ref Body2.Data, NormalWS, Friction);
        if ((UsageMask & 0b0100) != 0) Contact2.SolveVelocityFrictionConstraints(ref Body1.Data, ref Body2.Data, NormalWS, Friction);
        if ((UsageMask & 0b1000) != 0) Contact3.SolveVelocityFrictionConstraints(ref Body1.Data, ref Body2.Data, NormalWS, Friction);

        // Then apply all non-penetration constraints
        if ((UsageMask & 0b0001) != 0) Contact0.SolveVelocityNonPenetrationConstraints(ref Body1.Data, ref Body2.Data, NormalWS);
        if ((UsageMask & 0b0010) != 0) Contact1.SolveVelocityNonPenetrationConstraints(ref Body1.Data, ref Body2.Data, NormalWS);
        if ((UsageMask & 0b0100) != 0) Contact2.SolveVelocityNonPenetrationConstraints(ref Body1.Data, ref Body2.Data, NormalWS);
        if ((UsageMask & 0b1000) != 0) Contact3.SolveVelocityNonPenetrationConstraints(ref Body1.Data, ref Body2.Data, NormalWS);
    }

    public void SolvePositionConstraints()
    {
        if ((UsageMask & 0b0001) != 0) Contact0.SolvePositionConstraints(ref Body1.Data, ref Body2.Data, NormalWS);
        if ((UsageMask & 0b0010) != 0) Contact1.SolvePositionConstraints(ref Body1.Data, ref Body2.Data, NormalWS);
        if ((UsageMask & 0b0100) != 0) Contact2.SolvePositionConstraints(ref Body1.Data, ref Body2.Data, NormalWS);
        if ((UsageMask & 0b1000) != 0) Contact3.SolvePositionConstraints(ref Body1.Data, ref Body2.Data, NormalWS);
    }

    public int UpdatePosition()
    {
        int delc = 0;
        if ((UsageMask & 0b0001) != 0)
        {
            if (!Contact0.UpdatePosition(ref Body1.Data, ref Body2.Data, NormalWS))
            {
                delc++;
                UsageMask &= ~(1 << 0);
            }
        }

        if ((UsageMask & 0b0010) != 0)
        {
            if (!Contact1.UpdatePosition(ref Body1.Data, ref Body2.Data , NormalWS))
            {
                delc++;
                UsageMask &= ~(1 << 1);
            }
        }

        if ((UsageMask & 0b0100) != 0)
        {
            if (!Contact2.UpdatePosition(ref Body1.Data, ref Body2.Data, NormalWS))
            {
                delc++;
                UsageMask &= ~(1 << 2);
            }
        }

        if ((UsageMask & 0b1000) != 0)
        {
            if (!Contact3.UpdatePosition(ref Body1.Data, ref Body2.Data, NormalWS))
            {
                delc++;
                UsageMask &= ~(1 << 3);
            }
        }

        return delc;
    }

    public void Init(RigidBody body1, RigidBody body2)
    {
        Body1 = body1.handle;
        Body2 = body2.handle;

        Friction = MathF.Sqrt(body1.Friction * body2.Friction);
        Restitution = MathF.Max(body1.Restitution, body2.Restitution);

        UsageMask = 0;

        //Contacts = new List<Contact>(4);
    }

    // ---------------------------------------------------------------------------------------------------------
    //
    // The following contact caching code is heavily influenced / a direct copy of the great
    // Bullet Physics Engine:
    //
    // Bullet Continuous Collision Detection and Physics Library Copyright (c) 2003-2006 Erwin
    // Coumans  https://bulletphysics.org

    // This software is provided 'as-is', without any express or implied warranty. In no event will
    // the authors be held liable for any damages arising from the use of this software. Permission
    // is granted to anyone to use this software for any purpose, including commercial applications,
    // and to alter it and redistribute it freely, subject to the following restrictions:

    // 1. The origin of this software must not be misrepresented; you must not claim that you wrote
    //    the original software. If you use this software in a product, an acknowledgment in the
    //    product documentation would be appreciated but is not required.
    // 2. Altered source versions must be plainly marked as such, and must not be misrepresented as
    //    being the original software.
    // 3. This notice may not be removed or altered from any source distribution.
    //
    // https://github.com/bulletphysics/bullet3/blob/39b8de74df93721add193e5b3d9ebee579faebf8/
    // src/Bullet3OpenCL/NarrowphaseCollision/b3ContactCache.cpp

    public void AddContact(in JVector point1, in JVector point2, in JVector normal, float penetration)
    {
        throw new NotImplementedException();
    }

    /// <summary>
    /// Adds a new collision result to the contact manifold. Keeps at most four points.
    /// </summary>
    public void AddContact(in List<JVector> contactPoints1, in List<JVector> contactPoints2, in JVector normal, float stepTime)
    {
        NormalWS = normal;

        var contactPointCount = contactPoints1.Count;
        for (int i = 0; i < contactPointCount; i++)
        {
            var newPoint1 = contactPoints1[i];
            var newPoint2 = contactPoints2[i];

            // Check if we have a close contact point from last update
            var b1Position = Body1.Data.Position;
            var b2Position = Body2.Data.Position;

            JVector.Subtract(newPoint1, b1Position, out var newWorldRelPoint1);
            JVector.Subtract(newPoint2, b2Position, out var newWorldRelPoint2);
            JVector.ConjugatedTransform(newWorldRelPoint1, Body1.Data.Orientation, out var newLocalRelPos1);
            JVector.ConjugatedTransform(newWorldRelPoint2, Body2.Data.Orientation, out var newLocalRelPos2);

            if (UsageMask == 0b1111)
            {
                ref var cref = ref Contact0;
                int index = -1;
                if ((newLocalRelPos1 - Contact0.LocalRelPos1).LengthSquared() <= ContactPointPreserveLambdaMaxDistSq
                    && (newLocalRelPos2 - Contact0.LocalRelPos2).LengthSquared() <= ContactPointPreserveLambdaMaxDistSq)
                {
                    cref = ref Contact0;
                    index = 0;
                }
                if ((newLocalRelPos1 - Contact1.LocalRelPos1).LengthSquared() <= ContactPointPreserveLambdaMaxDistSq
                    && (newLocalRelPos2 - Contact1.LocalRelPos2).LengthSquared() <= ContactPointPreserveLambdaMaxDistSq)
                {
                    cref = ref Contact1;
                    index = 1;
                }
                if ((newLocalRelPos1 - Contact2.LocalRelPos1).LengthSquared() <= ContactPointPreserveLambdaMaxDistSq
                    && (newLocalRelPos2 - Contact2.LocalRelPos2).LengthSquared() <= ContactPointPreserveLambdaMaxDistSq)
                {
                    cref = ref Contact2;
                    index = 2;
                }
                if ((newLocalRelPos1 - Contact3.LocalRelPos1).LengthSquared() <= ContactPointPreserveLambdaMaxDistSq
                && (newLocalRelPos2 - Contact3.LocalRelPos2).LengthSquared() <= ContactPointPreserveLambdaMaxDistSq)
                {
                    cref = ref Contact3;
                    index = 3;
                }

                if (index != -1)
                {
                    cref.Initialize(ref Body1.Data, ref Body2.Data, newPoint1, newPoint2, normal, false, Restitution,
                        Friction, stepTime);
                    UsageMask |= 1 << index;
                }

                return;
            }


            if ((UsageMask & 0b0001) == 0)
            {
                Contact0.Initialize(ref Body1.Data, ref Body2.Data, newPoint1, newPoint2, normal, true, Restitution,
                    Friction, stepTime);
                UsageMask |= 1 << 0;
            }
            else if ((UsageMask & 0b0010) == 0)
            {
                Contact1.Initialize(ref Body1.Data, ref Body2.Data, newPoint1, newPoint2, normal, true, Restitution,
                    Friction, stepTime);
                UsageMask |= 1 << 1;
            }
            else if ((UsageMask & 0b0100) == 0)
            {
                Contact2.Initialize(ref Body1.Data, ref Body2.Data, newPoint1, newPoint2, normal, true, Restitution,
                    Friction, stepTime);
                UsageMask |= 1 << 2;
            }
            else if ((UsageMask & 0b1000) == 0)
            {
                Contact3.Initialize(ref Body1.Data, ref Body2.Data, newPoint1, newPoint2, normal, true, Restitution,
                    Friction, stepTime);
                UsageMask |= 1 << 3;
            }

        }
    }

    // ---------------------------------------------------------------------------------------------------------
    public struct Contact
    {
        public const float MaximumBias = 100.0f;
        public const float BiasFactor = 0.0f;
        public const float AllowedPenetration = 0.01f;
        public const float BreakThreshold = 0.02f;

        internal JVector LocalRelPos1;
        internal JVector LocalRelPos2;

        public JVector WorldRelPos1;
        public JVector WorldRelPos2;

        internal AxisConstraintPart NonPenetrationConstraint;
        internal AxisConstraintPart FrictionConstraint1;
        internal AxisConstraintPart FrictionConstraint2;


        public void Initialize(ref RigidBodyData b1, ref RigidBodyData b2, in JVector point1, in JVector point2, in JVector n,
            bool newContact, float combinedRestitution, float combinedFriction, float stepTime)
        {
            if (newContact)
            {
                NonPenetrationConstraint = new AxisConstraintPart();
                FrictionConstraint1 = new AxisConstraintPart();
                FrictionConstraint2 = new AxisConstraintPart();
            }
            else
            {
                var accImpulse = NonPenetrationConstraint.AccImpulse;
                NonPenetrationConstraint = new AxisConstraintPart();
                NonPenetrationConstraint.AccImpulse = accImpulse;

                accImpulse = FrictionConstraint1.AccImpulse;
                FrictionConstraint1 = new AxisConstraintPart();
                FrictionConstraint1.AccImpulse = accImpulse;

                accImpulse = FrictionConstraint2.AccImpulse;
                FrictionConstraint2 = new AxisConstraintPart();
                FrictionConstraint2.AccImpulse = accImpulse;
            }

            JVector.Subtract(point1, b1.Position, out WorldRelPos1);
            JVector.Subtract(point2, b2.Position, out WorldRelPos2);
            JVector.ConjugatedTransform(WorldRelPos1, b1.Orientation, out LocalRelPos1);
            JVector.ConjugatedTransform(WorldRelPos2, b2.Orientation, out LocalRelPos2);

            // Calculate collision points relative to body
            var p = 0.5f * (point1 + point2);
            var r1 = p - b1.Position;
            var r2 = p - b2.Position;

            // Calculate velocity of collision points
            JVector relativeVelocity = JVector.Zero;
            if (b1.MotionType != BodyMotionType.Static && b2.MotionType != BodyMotionType.Static)
            {
                relativeVelocity = b2.Velocity + b2.AngularVelocity % r2 - b1.Velocity + b1.AngularVelocity % r1;
            }
            else if (b1.MotionType != BodyMotionType.Static)
            {
                relativeVelocity = -b1.Velocity + b1.AngularVelocity % r1;
            }
            else if (b2.MotionType != BodyMotionType.Static)
            {
                relativeVelocity = b2.Velocity + b2.AngularVelocity % r2;
            }
            float normalVelocity = JVector.Dot(relativeVelocity, n);

            // How much the shapes are penetrating (> 0 if penetrating, < 0 if separated)
            float penetration = JVector.Dot(point1 - point2, n);

            // If there is no penetration, this is a speculative contact and we will apply a bias to the contact constraint
            // so that the constraint becomes relative_velocity . contact normal > -penetration / delta_time
            // instead of relative_velocity . contact normal > 0
            // See: GDC 2013: "Physics for Game Programmers; Continuous Collision" - Erin Catto
            float speculativeContactVelocityBias = MathF.Max(0.0f, -penetration / stepTime);

            // Determine if the velocity is big enough for restitution
            float normalVelocityBias = 0;
            if (combinedRestitution > 0.0f && normalVelocity < -1)
            {
                // We have a velocity that is big enough for restitution. This is where speculative contacts don't work
                // great as we have to decide now if we're going to apply the restitution or not. If the relative
                // velocity is big enough for a hit, we apply the restitution (in the end, due to other constraints,
                // the objects may actually not collide and we will have applied restitution incorrectly). Another
                // artifact that occurs because of this approximation is that the object will bounce from its current
                // position rather than from a position where it is touching the other object. This causes the object
                // to appear to move faster for 1 frame (the opposite of time stealing).
                if (normalVelocity < -speculativeContactVelocityBias)
                    normalVelocityBias = combinedRestitution * normalVelocity;
                else
                    // In this case we have predicted that we don't hit the other object, but if we do (due to other constraints changing velocities)
                    // the speculative contact will prevent penetration but will not apply restitution leading to another artifact.
                    normalVelocityBias = speculativeContactVelocityBias;
            }
            else
            {
                // No restitution. We can safely apply our contact velocity bias.
                normalVelocityBias = speculativeContactVelocityBias;
            }

            NonPenetrationConstraint.CalculateConstraintProperties(ref b1, ref b2, r1, r2, n, normalVelocityBias);

            var tangent1 = MathHelper.CreateOrthonormal(n);
            var tangent2 = n % tangent1;

            // Calculate friction part
            if (combinedFriction > 0.0f)
            {
                // Get surface velocity relative to tangents
                //JVector ws_surface_velocity = inSettings.mRelativeLinearSurfaceVelocity + inSettings.mRelativeAngularSurfaceVelocity.Cross(r1);
                float surface_velocity1 = 0;//inWorldSpaceTangent1.Dot(ws_surface_velocity);
                float surface_velocity2 = 0;// inWorldSpaceTangent2.Dot(ws_surface_velocity);

                // Implement friction as 2 AxisContraintParts
                FrictionConstraint1.CalculateConstraintProperties(ref b1, ref b2, r1, r2, tangent1, surface_velocity1);
                FrictionConstraint2.CalculateConstraintProperties(ref b1, ref b2, r1, r2, tangent2, surface_velocity2);
            }
            else
            {
                // Turn off friction constraint
                FrictionConstraint1.Deactivate();
                FrictionConstraint2.Deactivate();
            }
        }

        public bool UpdatePosition(ref RigidBodyData b1, ref RigidBodyData b2, JVector normalWS)
        {
            JVector.Transform(LocalRelPos1, b1.Orientation, out WorldRelPos1);
            JVector.Add(WorldRelPos1, b1.Position, out JVector p1);

            JVector.Transform(LocalRelPos2, b2.Orientation, out WorldRelPos2);
            JVector.Add(WorldRelPos2, b2.Position, out JVector p2);

            JVector.Subtract(p1, p2, out JVector dist);

            var penetration = JVector.Dot(dist, normalWS);

            if (penetration < -BreakThreshold * 0.1f)
            {
                return false;
            }

            dist -= penetration * normalWS;
            float tangentialOffsetSq = dist.LengthSquared();

            if (tangentialOffsetSq > BreakThreshold * BreakThreshold)
            {
                return false;
            }

            return true;
        }

        public void WarmStartVelocityConstraints(ref RigidBodyData b1, ref RigidBodyData b2, JVector normal,
            float idt, bool speculative = false)
        {
            var tangent1 = MathHelper.CreateOrthonormal(normal);
            var tangent2 = normal % tangent1;

            // Warm starting: Apply impulse from last frame
            if (FrictionConstraint1.IsActive() || FrictionConstraint2.IsActive())
            {
                FrictionConstraint1.WarmStart(ref b1, ref b2, tangent1, 1);
                FrictionConstraint2.WarmStart(ref b1, ref b2, tangent2, 1);
            }
            NonPenetrationConstraint.WarmStart(ref b1, ref b2, normal, 1);
        }

        public bool SolveVelocityFrictionConstraints(ref RigidBodyData b1, ref RigidBodyData b2, JVector normal, float friction)
        {
            bool any_impulse_applied = false;

            var tangent1 = MathHelper.CreateOrthonormal(normal);
            var tangent2 = normal % tangent1;

            // First apply all friction constraints (non-penetration is more important than friction)
            // Check if friction is enabled
            if (FrictionConstraint1.IsActive() || FrictionConstraint2.IsActive())
            {
                // Calculate impulse to stop motion in tangential direction
                float lambda1 = FrictionConstraint1.SolveVelocityConstraintGetAccImpulse(ref b1, ref b2, tangent1);
                float lambda2 = FrictionConstraint2.SolveVelocityConstraintGetAccImpulse(ref b1, ref b2, tangent2);
                float total_lambda_sq = lambda1 * lambda1 + lambda2 * lambda2;

                // Calculate max impulse that can be applied. Note that we're using the non-penetration impulse from the previous iteration here.
                // We do this because non-penetration is more important so is solved last (the last things that are solved in an iterative solver
                // contribute the most).
                float max_lambda_f = friction * NonPenetrationConstraint.AccImpulse;

                // If the total lambda that we will apply is too large, scale it back
                if (total_lambda_sq > max_lambda_f * max_lambda_f)
                {
                    float scale = max_lambda_f / MathF.Sqrt(total_lambda_sq);
                    lambda1 *= scale;
                    lambda2 *= scale;
                }

                // Apply the friction impulse
                if (FrictionConstraint1.SolveVelocityConstraintApplyLambda(ref b1, ref b2, tangent1, lambda1))
                {
                    any_impulse_applied = true;
                }
                if (FrictionConstraint2.SolveVelocityConstraintApplyLambda(ref b1, ref b2, tangent2, lambda2))
                {
                    any_impulse_applied = true;
                }
            }

            return any_impulse_applied;
        }

        public bool SolveVelocityNonPenetrationConstraints(ref RigidBodyData b1, ref RigidBodyData b2, JVector normal)
        {
            // Solve non penetration velocities
            return NonPenetrationConstraint.SolveVelocityConstraint(ref b1, ref b2, normal, 0.0f, float.MaxValue);
        }

        /// How much bodies are allowed to sink into each other (unit: meters)
        private const  float PenetrationSlop = 0.02f;
        /// Maximum distance to correct in a single iteration when solving position constraints (unit: meters)
        private const float MaxPenetrationDistance = 0.2f;
        /// Baumgarte stabilization factor (how much of the position error to 'fix' in 1 update) (unit: dimensionless, 0 = nothing, 1 = 100%)
        private const float Baumgarte = 0.2f;

        private void CalculateNonPenetrationConstraintProperties(ref RigidBodyData b1, ref RigidBodyData b2, JVector p1, JVector p2, JVector normalWS)
        {
            // Calculate collision points relative to body
            JVector p = 0.5f * (p1 + p2);
            JVector r1 = p - b1.Position;
            JVector r2 = p - b2.Position;

            NonPenetrationConstraint.CalculateConstraintProperties(ref b1, ref b2, r1, r2, normalWS, 0);
        }

        public bool SolvePositionConstraints(ref RigidBodyData b1, ref RigidBodyData b2, JVector normalWS)
        {
            bool any_impulse_applied = false;

            // Calculate new contact point positions in world space (the bodies may have moved)
            JVector.Transform(LocalRelPos1, b1.Orientation, out WorldRelPos1);
            JVector.Transform(LocalRelPos2, b2.Orientation, out WorldRelPos2);
            JVector.Add(WorldRelPos1, b1.Position, out JVector p1);
            JVector.Add(WorldRelPos2, b2.Position, out JVector p2);

            // Calculate separation along the normal (negative if interpenetrating)
            // Allow a little penetration by default (PhysicsSettings::mPenetrationSlop) to avoid jittering between contact/no-contact which wipes out the contact cache and warm start impulses
            // Clamp penetration to a max PhysicsSettings::mMaxPenetrationDistance so that we don't apply a huge impulse if we're penetrating a lot
            float separation = MathF.Max(JVector.Dot(p2 - p1, normalWS) + PenetrationSlop, -MaxPenetrationDistance);

            // Only enforce constraint when separation < 0 (otherwise we're apart)
            if (separation < 0.0f)
            {
                // Update constraint properties (bodies may have moved)
                CalculateNonPenetrationConstraintProperties(ref b1, ref b2, p1, p2, normalWS);

                // Solve position errors
                if (NonPenetrationConstraint.SolvePositionConstraintWithMassOverride(ref b1, ref b2, normalWS, separation, Baumgarte))
                    any_impulse_applied = true;
            }

            return any_impulse_applied;
        }
    }
}