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
using System.Text.RegularExpressions;
using Jitter2.LinearMath;

namespace Jitter2.Collision.Shapes;

/// <summary>
/// Represents a cylinder shape.
/// </summary>
public class CylinderShape : Shape
{
    // Approximation of top face with 8 vertices
    private static float cSin45 = 0.70710678118654752440084436210485f;
    private static JVector[] cTopFace = new JVector[8] 
    {
        new JVector(0.0f,      1.0f,   1.0f),
        new JVector(cSin45,    1.0f,   cSin45),
        new JVector(1.0f,      1.0f,   0.0f),
        new JVector(cSin45,    1.0f,   -cSin45),
        new JVector(-0.0f,     1.0f,   -1.0f),
        new JVector(-cSin45,   1.0f,   -cSin45),
        new JVector(-1.0f,     1.0f,   0.0f),
        new JVector(-cSin45,   1.0f,   cSin45)
    };


    private float radius;
    private float height;

    /// <summary>
    /// Gets or sets the radius of the cylinder.
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
    /// Gets or sets the height of the cylinder.
    /// </summary>
    public float Height
    {
        get => height;
        set
        {
            height = value;
            UpdateShape();
        }
    }

    /// <summary>
    /// Initializes a new instance of the <see cref="CylinderShape"/> class, creating a cylinder shape with the specified height and radius. The symmetry axis of the cylinder is aligned along the y-axis.
    /// </summary>
    /// <param name="height">The height of the cylinder.</param>
    /// <param name="radius">The radius of the cylinder at its base.</param>
    public CylinderShape(float height, float radius)
    {
        this.radius = radius;
        this.height = height;
        UpdateShape();
    }

    public override void SupportMap(in JVector direction, out JVector result)
    {
        float sigma = (float)Math.Sqrt(direction.X * direction.X + direction.Z * direction.Z);

        if (sigma > 0.0f)
        {
            result.X = direction.X / sigma * radius;
            result.Y = Math.Sign(direction.Y) * height * 0.5f;
            result.Z = direction.Z / sigma * radius;
        }
        else
        {
            result.X = 0.0f;
            result.Y = Math.Sign(direction.Y) * height * 0.5f;
            result.Z = 0.0f;
        }
    }

    public override void SupportingFace(in JVector inDirection, in JMatrix transform, in JVector position, out List<JVector> outVertices)
    {
        // Get scaled cylinder
        float halfHeight = 0.5f * height;

        float x = inDirection.X, y = inDirection.Y, z = inDirection.Z;
        float o = MathF.Sqrt(x * x + z * z);

        outVertices = new List<JVector>();

        // If o / |y| > scaled_radius / scaled_half_height, we're hitting the side
        if (o * halfHeight > radius * MathF.Abs(y))
        {
            // Hitting side
            float f = -radius / o;
            float vx = x * f;
            float vz = z * f;

            JVector.Transform(new JVector(vx, halfHeight, vz), transform, out var temp);
            JVector.Add(temp, position, out temp);
            outVertices.Add(temp);

            JVector.Transform(new JVector(vx, -halfHeight, vz), transform, out temp);
            JVector.Add(temp, position, out temp);
            outVertices.Add(temp);
        }
        else
        {
            // Hitting top or bottom
            var multiplier = y < 0.0f ? new JVector(radius, halfHeight, radius) : new JVector(-radius, -halfHeight, radius);
            var transformTemp = JMatrix.PreScaled(transform, multiplier);
            foreach(var v in cTopFace)
            {
                var temp = JVector.Transform(v, transformTemp);
                JVector.Add(temp, position, out temp);
                outVertices.Add(temp);
            }
        }
    }

    public override JVector SurfaceNormal(JVector inLocalSurfacePosition)
    {
        float halfHeight = 0.5f * height;

        // Calculate distance to infinite cylinder surface
        var local_surface_position_xz = new JVector(inLocalSurfacePosition.X, 0, inLocalSurfacePosition.Z);
        float local_surface_position_xz_len = local_surface_position_xz.Length();
        float distance_to_curved_surface = MathF.Abs(local_surface_position_xz_len - radius);

        // Calculate distance to top or bottom plane
        float distance_to_top_or_bottom = MathF.Abs(MathF.Abs(inLocalSurfacePosition.Y) - halfHeight);

        // Return normal according to closest surface
        if (distance_to_curved_surface < distance_to_top_or_bottom)
            return JVector.Multiply(local_surface_position_xz, 1/local_surface_position_xz_len);
        else
            return inLocalSurfacePosition.Y > 0.0f ? new JVector(0, 1, 0) : -new JVector(0, 1, 0);
    }


    public override void CalculateBoundingBox(in JQuaternion orientation, in JVector position, out JBBox box)
    {
        const float ZeroEpsilon = 1e-12f;

        JVector upa = orientation.GetBasisY();

        float xx = upa.X * upa.X;
        float yy = upa.Y * upa.Y;
        float zz = upa.Z * upa.Z;

        float l1 = yy + zz;
        float l2 = xx + zz;
        float l3 = xx + yy;

        float xext = 0, yext = 0, zext = 0;

        if (l1 > ZeroEpsilon)
        {
            float sl = 1.0f / MathF.Sqrt(l1);
            xext = (yy + zz) * sl * radius;
        }

        if (l2 > ZeroEpsilon)
        {
            float sl = 1.0f / MathF.Sqrt(l2);
            yext = (xx + zz) * sl * radius;
        }

        if (l3 > ZeroEpsilon)
        {
            float sl = 1.0f / MathF.Sqrt(l3);
            zext = (xx + yy) * sl * radius;
        }

        JVector p1 = -0.5f * height * upa;
        JVector p2 = +0.5f * height * upa;

        JVector delta = JVector.Max(p1, p2) + new JVector(xext, yext, zext);

        box.Min = position - delta;
        box.Max = position + delta;
    }

    public override void CalculateMassInertia(out JMatrix inertia, out JVector com, out float mass)
    {
        mass = MathF.PI * radius * radius * height;

        inertia = JMatrix.Identity;
        inertia.M11 = 1.0f / 4.0f * mass * radius * radius + 1.0f / 12.0f * mass * height * height;
        inertia.M22 = 1.0f / 2.0f * mass * radius * radius;
        inertia.M33 = 1.0f / 4.0f * mass * radius * radius + 1.0f / 12.0f * mass * height * height;

        com = JVector.Zero;
    }
}