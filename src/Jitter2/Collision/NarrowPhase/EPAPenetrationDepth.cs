using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace Jitter2.Collision
{
    /// <summary>
    /// Implementation of Expanding Polytope Algorithm as described in:
    ///
    /// Proximity Queries and Penetration Depth Computation on 3D Game Objects - Gino van den Bergen
    ///
    /// The implementation of this algorithm does not completely follow the article, instead of splitting
    /// triangles at each edge as in fig. 7 in the article, we build a convex hull (removing any triangles that
    /// are facing the new point, thereby avoiding the problem of getting really oblong triangles as mentioned in
    /// the article).
    ///
    /// The algorithm roughly works like:
    ///
    /// - Start with a simplex of the Minkowski sum (difference) of two objects that was calculated by GJK
    /// - This simplex should contain the origin (or else GJK would have reported: no collision)
    /// - In cases where the simplex consists of 1 - 3 points, find some extra support points (of the Minkowski sum) to get to at least 4 points
    /// - Convert this into a convex hull with non-zero volume (which includes the origin)
    /// - A: Calculate the closest point to the origin for all triangles of the hull and take the closest one
    /// - Calculate a new support point (of the Minkowski sum) in this direction and add this point to the convex hull
    /// - This will remove all faces that are facing the new point and will create new triangles to fill up the hole
    /// - Loop to A until no closer point found
    /// - The closest point indicates the position / direction of least penetration
    /// </summary>
    internal class EPAPenetrationDepth
    {



    }
}
