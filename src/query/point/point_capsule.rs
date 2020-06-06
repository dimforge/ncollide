use crate::math::{Isometry, Point, Vector};
use crate::query::{PointProjection, PointQuery};
use crate::shape::{Capsule, FeatureId, Segment};
use na::{self, RealField, Unit};

impl<N: RealField> PointQuery<N> for Capsule<N> {
    #[inline]
    fn project_point(&self, m: &Isometry<N>, pt: &Point<N>, solid: bool) -> PointProjection<N> {
        let mut y = Point::origin();
        y[1] = self.half_height;
        let seg = Segment::new(-y, y);
        let proj = seg.project_point(m, pt, solid);
        let dproj = *pt - proj.point;

        if let Some((dir, dist)) = Unit::try_new_and_get(dproj, N::default_epsilon()) {
            let inside = dist <= self.radius;
            if solid && inside {
                PointProjection::new(true, *pt)
            } else {
                PointProjection::new(inside, proj.point + dir.into_inner() * self.radius)
            }
        } else {
            if solid {
                PointProjection::new(true, *pt)
            } else {
                let mut dir: Vector<N> = na::zero();
                dir[1] = na::one();
                dir = m * dir;
                PointProjection::new(true, proj.point + dir * self.radius)
            }
        }
    }

    #[inline]
    fn project_point_with_feature(
        &self,
        m: &Isometry<N>,
        pt: &Point<N>,
    ) -> (PointProjection<N>, FeatureId) {
        (self.project_point(m, pt, false), FeatureId::Face(0))
    }
}
