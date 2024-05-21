using Jitter2.LinearMath;
using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Jitter2.Geometry
{
    internal static class ClipPoly
    {
        /// Clip inPolygonToClip against the positive halfspace of plane defined by inPlaneOrigin and inPlaneNormal.
        /// inPlaneNormal does not need to be normalized.
        public static void ClipPolyVsPlane(List<JVector> inPolygonToClip, JVector inPlaneOrigin, JVector inPlaneNormal, List<JVector> outClippedPolygon)
        {
            Debug.Assert(inPolygonToClip.Count >= 2);
            Debug.Assert(outClippedPolygon.Count < 1);

            // Determine state of last point
            var e1 = inPolygonToClip[inPolygonToClip.Count - 1];
            float prev_num = JVector.Dot(inPlaneOrigin - e1, inPlaneNormal);
            bool prev_inside = prev_num < 0.0f;

            // Loop through all vertices
            for (var j = 0; j < inPolygonToClip.Count; ++j)
            {
                // Check if second point is inside
                JVector e2 = inPolygonToClip[j];
                float num = JVector.Dot(inPlaneOrigin - e2, inPlaneNormal);
                bool cur_inside = num < 0.0f;

                // In -> Out or Out -> In: Add point on clipping plane
                if (cur_inside != prev_inside)
                {
                    // Solve: (X - inPlaneOrigin) . inPlaneNormal = 0 and X = e1 + t * (e2 - e1) for X
                    JVector e12 = e2 - e1;
                    float denom = JVector.Dot(e12, inPlaneNormal);
                    if (denom != 0.0f)
                        outClippedPolygon.Add(e1 + (prev_num / denom) * e12);
                    else
                        cur_inside = prev_inside; // Edge is parallel to plane, treat point as if it were on the same side as the last point
                }

                // Point inside, add it
                if (cur_inside)
                    outClippedPolygon.Add(e2);

                // Update previous state
                prev_num = num;
                prev_inside = cur_inside;
                e1 = e2;
            }
        }


        /// Clip polygon versus polygon.
        /// Both polygons are assumed to be in counter clockwise order.
        public static void ClipPolyVsPoly(List<JVector> inPolygonToClip, List<JVector> inClippingPolygon, JVector inClippingPolygonNormal, List<JVector> outClippedPolygon)
        {
            Debug.Assert(inPolygonToClip.Count >= 2);
            Debug.Assert(inClippingPolygon.Count >= 3);

            List<JVector>[] tmp_vertices = new List<JVector>[2] { new List<JVector>(), new List<JVector>() };
            int tmp_vertices_idx = 0;
            for (int i = 0; i < inClippingPolygon.Count; ++i)
            {
                // Get edge to clip against
                JVector clip_e1 = inClippingPolygon[i];
                JVector clip_e2 = inClippingPolygon[(i + 1) % inClippingPolygon.Count];
                JVector clip_normal = JVector.Cross(inClippingPolygonNormal, (clip_e2 - clip_e1)); // Pointing inward to the clipping polygon

                // Get source and target polygon
                var src_polygon = (i == 0) ? inPolygonToClip : tmp_vertices[tmp_vertices_idx];
                tmp_vertices_idx ^= 1;
                var tgt_polygon = (i == inClippingPolygon.Count - 1) ? outClippedPolygon : tmp_vertices[tmp_vertices_idx];
                tgt_polygon.Clear();

                // Clip against the edge
                ClipPolyVsPlane(src_polygon, clip_e1, clip_normal, tgt_polygon);

                // Break out if no polygon left
                if (tgt_polygon.Count < 3)
                {
                    outClippedPolygon.Clear();
                    break;
                }
            }
        }


        /// Clip inPolygonToClip against an edge, the edge is projected on inPolygonToClip using inClippingEdgeNormal.
        /// The positive half space (the side on the edge in the direction of inClippingEdgeNormal) is cut away.
        public static void ClipPolyVsEdge(List<JVector> inPolygonToClip, JVector inEdgeVertex1, JVector inEdgeVertex2, JVector inClippingEdgeNormal, List<JVector> outClippedPolygon)
        {
            Debug.Assert(inPolygonToClip.Count >= 3);
            Debug.Assert(outClippedPolygon.Count < 0);

            // Get normal that is perpendicular to the edge and the clipping edge normal
            JVector edge = inEdgeVertex2 - inEdgeVertex1;
            JVector edge_normal = JVector.Cross(inClippingEdgeNormal, edge);

            // Project vertices of edge on inPolygonToClip
            JVector polygon_normal = JVector.Cross(inPolygonToClip[2] - inPolygonToClip[0], inPolygonToClip[1] - inPolygonToClip[0]);
            float polygon_normal_len_sq = polygon_normal.LengthSquared();
            JVector v1 = inEdgeVertex1 + JVector.Dot(polygon_normal, inPolygonToClip[0] - inEdgeVertex1) * polygon_normal * (1 / polygon_normal_len_sq);
            JVector v2 = inEdgeVertex2 + JVector.Dot(polygon_normal, inPolygonToClip[0] - inEdgeVertex2) * polygon_normal * (1 / polygon_normal_len_sq);
            JVector v12 = v2 - v1;
            float v12_len_sq = v12.LengthSquared();

            // Determine state of last point
            JVector e1 = inPolygonToClip[inPolygonToClip.Count - 1];
            float prev_num = JVector.Dot(inEdgeVertex1 - e1, edge_normal);
            bool prev_inside = prev_num < 0.0f;

            // Loop through all vertices
            for (var j = 0; j < inPolygonToClip.Count; ++j)
            {
                // Check if second point is inside
                JVector e2 = inPolygonToClip[j];
                float num = JVector.Dot(inEdgeVertex1 - e2, edge_normal);
                bool cur_inside = num < 0.0f;

                // In -> Out or Out -> In: Add point on clipping plane
                if (cur_inside != prev_inside)
                {
                    // Solve: (X - inPlaneOrigin) . inPlaneNormal = 0 and X = e1 + t * (e2 - e1) for X
                    JVector e12 = e2 - e1;
                    float denom = JVector.Dot(e12, edge_normal);
                    JVector clipped_point = e1 + (prev_num / denom) * e12;

                    // Project point on line segment v1, v2 so see if it falls outside if the edge
                    float projection = JVector.Dot(clipped_point - v1, v12);
                    if (projection < 0.0f)

                        outClippedPolygon.Add(v1);
                    else if (projection > v12_len_sq)
                        outClippedPolygon.Add(v2);
                    else
                        outClippedPolygon.Add(clipped_point);
                }

                // Update previous state
                prev_num = num;
                prev_inside = cur_inside;
                e1 = e2;
            }
        }


        /// Clip polygon vs axis aligned box, inPolygonToClip is assume to be in counter clockwise order.
        /// Output will be stored in outClippedPolygon. Everything inside inAABox will be kept.
        public static void ClipPolyVsAABox(List<JVector> inPolygonToClip, JBBox inAABox, List<JVector> outClippedPolygon)
        {
            Debug.Assert(inPolygonToClip.Count >= 2);

            List<JVector>[] tmp_vertices = new List<JVector>[2] { new List<JVector>(), new List<JVector>() };
            int tmp_vertices_idx = 0;

            for (int coord = 0; coord < 3; ++coord)
                for (int side = 0; side < 2; ++side)
                {
                    // Get plane to clip against
                    JVector origin = JVector.Zero, normal = JVector.Zero;
                    if (side == 0)
                    {
                        normal[coord] = 1.0f;
                        origin[coord] = inAABox.Min[coord];
                    }
                    else
                    {
                        normal[coord] = -1.0f;
                        origin[coord] = inAABox.Max[coord];
                    }

                    // Get source and target polygon
                    var src_polygon = tmp_vertices_idx == 0 ? inPolygonToClip : tmp_vertices[tmp_vertices_idx & 1];
                    tmp_vertices_idx++;
                    var tgt_polygon = tmp_vertices_idx == 6 ? outClippedPolygon : tmp_vertices[tmp_vertices_idx & 1];
                    tgt_polygon.Clear();

                    // Clip against the edge
                    ClipPolyVsPlane(src_polygon, origin, normal, tgt_polygon);

                    // Break out if no polygon left
                    if (tgt_polygon.Count() < 3)
                    {
                        outClippedPolygon.Count();
                        return;
                    }

                    // Flip normal
                    normal = -normal;
                }
        }


    }
}
