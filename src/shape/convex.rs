
#[deriving(PartialEq, Show, Clone, RustcEncodable, RustcDecodable)]
/// The implicit convex hull of a set of points.
pub struct Convex<P> {
    points: Vec<P>
}

impl<P> Convex<P> {
    /// Creates a polytope from a set of point.
    ///
    /// The set of point as not assumed to form a convex polytope.
    #[inline]
    pub fn new(points: Vec<P>) -> Convex<P> {
        Convex {
            points: points
        }
    }
}

impl<P> Convex<P> {
    /// The list of points of this convex polytope.
    #[inline]
    pub fn points(&self) -> &[P] { // FIXME: naming: `points` vs. `points`?
        self.points.as_slice()
    }
}
