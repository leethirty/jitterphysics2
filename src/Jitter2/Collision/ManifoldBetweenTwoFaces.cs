using Jitter2.Geometry;
using Jitter2.LinearMath;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Jitter2.Collision
{
    internal static class ManifoldBetweenTwoFacesHelper
    {

        /// Remove contact points if there are > 4 (no more than 4 are needed for a stable solution)
        public static void PruneContactPoints()
        {

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
