using Jitter2.Geometry;
using Jitter2.LinearMath;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Jitter2.Collision
{
    internal static class ManifoldBetweenTwoFacesHelper
    {

        /// Remove contact points if there are > 4 (no more than 4 are needed for a stable solution)
        public static void PruneContactPoints(JVector inPenetrationAxis, List<JVector> ioContactPointsOn1, List<JVector> ioContactPointsOn2)
        {
            // Makes no sense to call this with 4 or less points
            Debug.Assert(ioContactPointsOn1.Count > 4);

            // Both arrays should have the same size
            Debug.Assert(ioContactPointsOn1.Count == ioContactPointsOn2.Count);

            // Penetration axis must be normalized
            Debug.Assert(MathF.Abs(inPenetrationAxis.LengthSquared() - 1) <= 0.0000009f);

            // We use a heuristic of (distance to center of mass) * (penetration depth) to find the contact point that we should keep
            // Neither of those two terms should ever become zero, so we clamp against this minimum value
            const float cMinDistanceSq = 1.0e-6f; // 1 mm

            List<JVector> projected = new List<JVector>();
            List<float> penetration_depth_sq = new List<float>();
            for (int i = 0; i < ioContactPointsOn1.Count; ++i)
            {
                // Project contact points on the plane through inCenterOfMass with normal inPenetrationAxis and center around the center of mass of body 1
                // (note that since all points are relative to inCenterOfMass we can project onto the plane through the origin)
                var v1 = ioContactPointsOn1[i];
                projected.Add(v1 - JVector.Dot(v1, inPenetrationAxis) * inPenetrationAxis);

                // Calculate penetration depth^2 of each point and clamp against the minimal distance
                var v2 = ioContactPointsOn2[i];
                penetration_depth_sq.Add(MathF.Max(cMinDistanceSq, (v2 - v1).LengthSquared()));
            }

            // Find the point that is furthest away from the center of mass (its torque will have the biggest influence)
            // and the point that has the deepest penetration depth. Use the heuristic (distance to center of mass) * (penetration depth) for this.
            int point1 = 0;
            float val = MathF.Max(cMinDistanceSq, projected[0].LengthSquared()) * penetration_depth_sq[0];
            for (int i = 0; i < projected.Count; ++i)
            {
                float v = MathF.Max(cMinDistanceSq, projected[i].LengthSquared()) * penetration_depth_sq[i];
                if (v > val)
                {
                    val = v;
                    point1 = i;
                }
            }
            var point1v = projected[point1];

            // Find point furthest from the first point forming a line segment with point1. Again combine this with the heuristic
            // for deepest point as per above.
            int point2 = -1;
            val = -float.MaxValue;
            for (int i = 0; i < projected.Count; ++i)
                if (i != point1)
                {
                    float v = MathF.Max(cMinDistanceSq, (projected[i] - point1v).LengthSquared()) * penetration_depth_sq[i];
                    if (v > val)
                    {
                        val = v;
                        point2 = i;
                    }
                }
            Debug.Assert(point2 != -1);
            var point2v = projected[point2];

            // Find furthest points on both sides of the line segment in order to maximize the area
            int point3 = -1;
            int point4 = -1;
            float min_val = 0.0f;
            float max_val = 0.0f;
            var perp = JVector.Cross((point2v - point1v), (inPenetrationAxis));
            for (int i = 0; i < projected.Count; ++i)
                if (i != point1 && i != point2)
                {
                    float v = JVector.Dot( perp, projected[i] - point1v);
                    if (v < min_val)
                    {
                        min_val = v;
                        point3 = i;
                    }
                    else if (v > max_val)
                    {
                        max_val = v;
                        point4 = i;
                    }
                }

            // Add points to array (in order so they form a polygon)
            List<JVector> points_to_keep_on_1 = new List<JVector>();
            List<JVector> points_to_keep_on_2 = new List<JVector>();
            points_to_keep_on_1.Add(ioContactPointsOn1[point1]);
            points_to_keep_on_2.Add(ioContactPointsOn2[point1]);
            if (point3 != -1)
            {
                points_to_keep_on_1.Add(ioContactPointsOn1[point3]);
                points_to_keep_on_2.Add(ioContactPointsOn2[point3]);
            }
            points_to_keep_on_1.Add(ioContactPointsOn1[point2]);
            points_to_keep_on_2.Add(ioContactPointsOn2[point2]);
            if (point4 != -1)
            {
                Debug.Assert(point3 != point4);
                points_to_keep_on_1.Add(ioContactPointsOn1[point4]);
                points_to_keep_on_2.Add(ioContactPointsOn2[point4]);
            }

            // Copy the points back to the input buffer
            ioContactPointsOn1.Clear();
            ioContactPointsOn2.Clear();

            ioContactPointsOn1.AddRange(points_to_keep_on_1);
            ioContactPointsOn2.AddRange(points_to_keep_on_2);
        }

        /// Determine contact points between 2 faces of 2 shapes and return them in outContactPoints 1 & 2
        public static void ManifoldBetweenTwoFaces(JVector inContactPoint1, JVector inContactPoint2, JVector inPenetrationAxis, float inMaxContactDistanceSq, in List<JVector> inShape1Face,
            in List<JVector> inShape2Face, List<JVector> outContactPoints1, List<JVector> outContactPoints2)
        {
            // Remember size before adding new points, to check at the end if we added some
            var old_size = outContactPoints1.Count;

            // Check if both shapes have polygon faces
            if (inShape1Face.Count >= 2 // The dynamic shape needs to have at least 2 points or else there can never be more than 1 contact point
                && inShape2Face.Count >= 3) // The dynamic/static shape needs to have at least 3 points (in the case that it has 2 points only if the edges match exactly you can have 2 contact points, but this situation is unstable anyhow)
            {
                // Clip the polygon of face 2 against that of 1
                List<JVector> clipped_face = new List<JVector>();
                if (inShape1Face.Count >= 3)
                    ClipPoly.ClipPolyVsPoly(inShape2Face, inShape1Face, inPenetrationAxis, clipped_face);
                else if (inShape1Face.Count == 2)
                    ClipPoly.ClipPolyVsEdge(inShape2Face, inShape1Face[0], inShape1Face[1], inPenetrationAxis, clipped_face);

                // Project the points back onto the plane of shape 1 face and only keep those that are behind the plane
                JVector plane_origin = inShape1Face[0];
                JVector plane_normal;
                JVector first_edge = inShape1Face[1] - plane_origin;
                if (inShape1Face.Count >= 3)
                {
                    // Three vertices, can just calculate the normal
                    plane_normal = JVector.Cross(first_edge, inShape1Face[2] - plane_origin);
                }
                else
                {
                    // Two vertices, first find a perpendicular to the edge and penetration axis and then use the perpendicular together with the edge to form a normal
                    plane_normal = JVector.Cross(JVector.Cross(first_edge, inPenetrationAxis), first_edge);
                }

                // Check if the plane normal has any length, if not the clipped shape is so small that we'll just use the contact points
                float plane_normal_len_sq = plane_normal.LengthSquared();
                if (plane_normal_len_sq > 0.0f)
                {
                    // Discard points of faces that are too far away to collide
                    foreach (JVector p2 in clipped_face)
                    {
                        float distance = JVector.Dot(p2 - plane_origin, plane_normal); // Note should divide by length of plane_normal (unnormalized here)
                        if (distance <= 0.0f || distance * distance < inMaxContactDistanceSq * plane_normal_len_sq) // Must be close enough to plane, note we correct for not dividing by plane normal length here
                        {
                            // Project point back on shape 1 using the normal, note we correct for not dividing by plane normal length here:
                            // p1 = p2 - (distance / sqrt(plane_normal_len_sq)) * (plane_normal / sqrt(plane_normal_len_sq));
                            JVector p1 = p2 - (distance / plane_normal_len_sq) * plane_normal;

                            outContactPoints1.Add(p1);
                            outContactPoints2.Add(p2);
                        }
                    }
                }
            }

            // If the clipping result is empty, use the contact point itself
            if (outContactPoints1.Count == old_size)
            {
                outContactPoints1.Add(inContactPoint1);
                outContactPoints2.Add(inContactPoint2);
            }

        }

    }
}
