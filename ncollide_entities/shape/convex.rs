
#[derive(PartialEq, Debug, Clone, RustcEncodable, RustcDecodable)]
/// The implicit convex hull of a set of points.
pub struct ConvexHull<P> {
    points: Vec<P>
}

impl<P> ConvexHull<P> {
    /// Creates a polytope from a set of point.
    ///
    /// The set of point as not assumed to form a convex polytope.
    #[inline]
    pub fn new(points: Vec<P>) -> ConvexHull<P> {
        ConvexHull {
            points: points
        }
    }
}

impl<P> ConvexHull<P> {
    /// The list of points of this convex polytope.
    #[inline]
    pub fn points(&self) -> &[P] { // FIXME: naming: `points` vs. `points`?
        &self.points[..]
    }
}
