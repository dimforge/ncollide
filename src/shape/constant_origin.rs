use na::{RealField, Unit};

use crate::math::{Isometry, Point, Vector};
use crate::shape::SupportMap;

pub struct ConstantOrigin;

impl<N: RealField> SupportMap<N> for ConstantOrigin {
    fn support_point(&self, _: &Isometry<N>, _: &Vector<N>) -> Point<N> {
        Point::origin()
    }

    /// Same as `self.support_point` except that `dir` is normalized.
    fn support_point_toward(&self, _: &Isometry<N>, _: &Unit<Vector<N>>) -> Point<N> {
        Point::origin()
    }
}
