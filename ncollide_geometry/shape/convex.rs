use utils;

use shape::SupportMap;
use math::{Isometry, Point};

#[derive(PartialEq, Debug, Clone, RustcEncodable, RustcDecodable)]
/// The implicit convex hull of a set of points.
pub struct ConvexHull<P> {
    points: Vec<P>,
}

impl<P> ConvexHull<P> {
    /// Creates a polytope from a set of point.
    ///
    /// The set of point as not assumed to form a convex polytope.
    #[inline]
    pub fn new(points: Vec<P>) -> ConvexHull<P> {
        ConvexHull { points: points }
    }
}

impl<P> ConvexHull<P> {
    /// The list of points of this convex polytope.
    #[inline]
    pub fn points(&self) -> &[P] {
        &self.points[..]
    }
}

impl<P: Point, M: Isometry<P>> SupportMap<P, M> for ConvexHull<P> {
    #[inline]
    fn support_point(&self, m: &M, dir: &P::Vector) -> P {
        let local_dir = m.inverse_rotate_vector(dir);
        let best_pt = utils::point_cloud_support_point(&local_dir, self.points());

        m.transform_point(&best_pt)
    }
}
