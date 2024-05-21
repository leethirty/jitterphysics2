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
using static System.Formats.Asn1.AsnWriter;

namespace Jitter2.Collision.Shapes;

/// <summary>
/// Represents a shape in the form of a capsule.
/// </summary>
public class CapsuleShape : Shape
{
    /// Used by (Tapered)CapsuleShape to determine when supporting face is an edge rather than a point (unit: meter)
    private static float cCapsuleProjectionSlop = 0.02f;

    private float radius;
    private float halfLength;

    /// <summary>
    /// Gets or sets the radius of the capsule.
    /// </summary>
    public float Radius
    {
        get => radius;
        set
        {
            radius = value;
            UpdateShape();
        }
    }

    /// <summary>
    /// Gets or sets the length of the cylindrical part of the capsule, excluding the half-spheres on both ends.
    /// </summary>
    public float Length
    {
        get => 2.0f * halfLength;
        set
        {
            halfLength = value / 2.0f;
            UpdateShape();
        }
    }

    /// <summary>
    /// Initializes a new instance of the CapsuleShape class with the specified radius and length. The symmetry axis of the capsule is aligned along the Y-axis.
    /// </summary>
    /// <param name="radius">The radius of the capsule.</param>
    /// <param name="length">The length of the cylindrical part of the capsule, excluding the half-spheres at both ends.</param>
    public CapsuleShape(float radius = 0.5f, float length = 1.0f)
    {
        this.radius = radius;
        halfLength = 0.5f * length;
        UpdateShape();
    }

    public override void SupportMap(in JVector direction, out JVector result)
    {
        // capsule = segment + sphere

        // sphere
        JVector.Normalize(direction, out JVector ndir);
        result = ndir * radius;

        // two endpoint of the segment are
        // p_1 = (0, +length/2, 0)
        // p_2 = (0, -length/2, 0)

        // we have to calculate the dot-product with the direction
        // vector to decide whether p_1 or p_2 is the correct support point
        result.Y += MathF.Sign(direction.Y) * halfLength;
    }

    public override void SupportingFace(in JVector inDirection, in JMatrix orientation, in JVector position, out List<JVector> outVertices)
    {
        // Get direction in horizontal plane
        var direction = inDirection;
        direction[1] = 0;

        outVertices = new List<JVector>();

        // Check zero vector, in this case we're hitting from top/bottom so there's no supporting face
        float len = direction.Length();
        if (len == 0.0f)
            return;

        var halfHeight = new JVector(0, halfLength, 0);

        // Get support point for top and bottom sphere in the opposite of 'direction' (including convex radius)
        var support = (radius / len) * direction;
        var support_top = halfHeight - support;
        var support_bottom = -halfHeight - support;

        // Get projection on inDirection
        // Note that inDirection is not normalized, so we need to divide by inDirection.Length() to get the actual projection
        // We've multiplied both sides of the if below with inDirection.Length()
        float proj_top = JVector.Dot(support_top, inDirection);
        float proj_bottom = JVector.Dot(support_bottom, inDirection);

        // If projection is roughly equal then return line, otherwise we return nothing as there's only 1 point
        if (MathF.Abs(proj_top - proj_bottom) < cCapsuleProjectionSlop * inDirection.Length())
        {
            JVector.Transform(support_top, orientation, out var temp);
            JVector.Add(temp, position, out temp);
            outVertices.Add(temp);

            JVector.Transform(support_bottom, orientation, out temp);
            JVector.Add(temp, position, out temp);
            outVertices.Add(temp);
        }
    }

    public override JVector SurfaceNormal(JVector inLocalSurfacePosition)
    {
        if (inLocalSurfacePosition.Y > halfLength)
        {
            return JVector.Normalize(inLocalSurfacePosition - new JVector(0, halfLength, 0));
        }
        else if (inLocalSurfacePosition.Y < -halfLength)
        {
            return JVector.Normalize(inLocalSurfacePosition - new JVector(0, -halfLength, 0));
        }
        else
        {
            var temp = new JVector(inLocalSurfacePosition.X, 0, inLocalSurfacePosition.Z);
            if(temp.LengthSquared() > 0)
            {
                return JVector.Normalize(temp);
            }

            return new JVector(1, 0, 0);
        }
    }


    public override void CalculateBoundingBox(in JMatrix orientation, in JVector position, out JBBox box)
    {
        JVector delta = halfLength * orientation.GetColumn(1);

        box.Min.X = -radius - MathF.Abs(delta.X);
        box.Min.Y = -radius - MathF.Abs(delta.Y);
        box.Min.Z = -radius - MathF.Abs(delta.Z);

        box.Max.X = +radius + MathF.Abs(delta.X);
        box.Max.Y = +radius + MathF.Abs(delta.Y);
        box.Max.Z = +radius + MathF.Abs(delta.Z);

        box.Min += position;
        box.Max += position;
    }

    public override void CalculateMassInertia(out JMatrix inertia, out JVector com, out float mass)
    {
        float length = 2.0f * halfLength;

        float massSphere = 4.0f / 3.0f * MathF.PI * radius * radius * radius;
        float massCylinder = MathF.PI * radius * radius * length;

        inertia = JMatrix.Identity;

        inertia.M11 = massCylinder * (1.0f / 12.0f * length * length + 1.0f / 4.0f * radius * radius) + massSphere *
            (2.0f / 5.0f * radius * radius + 1.0f / 4.0f * length * length + 3.0f / 8.0f * length * radius);
        inertia.M22 = 1.0f / 2.0f * massCylinder * radius * radius + 2.0f / 5.0f * massSphere * radius * radius;
        inertia.M33 = massCylinder * (1.0f / 12.0f * length * length + 1.0f / 4.0f * radius * radius) + massSphere *
            (2.0f / 5.0f * radius * radius + 1.0f / 4.0f * length * length + 3.0f / 8.0f * length * radius);

        mass = massCylinder + massSphere;
        com = JVector.Zero;
    }
}