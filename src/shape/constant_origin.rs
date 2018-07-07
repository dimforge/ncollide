use na::{Real, Unit};

use shape::SupportMap;
use math::{Isometry, Point, Vector};

pub struct ConstantOrigin;

impl<N: Real> SupportMap<N> for ConstantOrigin {
    fn support_point(&self, _: &Isometry<N>, _: &Vector<N>) -> Point<N> {
        Point::origin()
    }

    /// Same as `self.support_point` except that `dir` is normalized.
    fn support_point_toward(&self, _: &Isometry<N>, _: &Unit<Vector<N>>) -> Point<N> {
        Point::origin()
    }

    fn local_support_point(&self, _: &Vector<N>) -> Point<N> {
        Point::origin()
    }

    /// Same as `self.support_point` except that `dir` is normalized.
    fn local_support_point_toward(&self, _: &Unit<Vector<N>>) -> Point<N> {
        Point::origin()
    }
}
