use approx::ApproxEq;
use na::{self, Unit};
use shape::{Capsule, FeatureId, Segment};
use query::{PointProjection, PointQuery};
use math::{Isometry, Point};

impl<P: Point, M: Isometry<P>> PointQuery<P, M> for Capsule<P::Real> {
    #[inline]
    fn project_point(&self, m: &M, pt: &P, solid: bool) -> PointProjection<P> {
        let mut y = P::origin();
        y[1] = self.half_height();
        let seg = Segment::new(-y, y);
        let proj = seg.project_point(m, pt, solid);
        let dproj = *pt - proj.point;

        if let Some((dir, dist)) = Unit::try_new_and_get(dproj, P::Real::default_epsilon()) {
            let inside = dist <= self.radius();
            if solid && inside {
                PointProjection::new(true, *pt)
            } else {
                PointProjection::new(inside, proj.point + dir.unwrap() * self.radius())
            }
        } else {
            if solid {
                PointProjection::new(true, *pt)
            } else {
                let mut dir: P::Vector = na::zero();
                dir[1] = na::one();
                dir = m.transform_vector(&dir);
                PointProjection::new(true, proj.point + dir * self.radius())
            }
        }
    }

    #[inline]
    fn project_point_with_feature(&self, m: &M, pt: &P) -> (PointProjection<P>, FeatureId) {
        (self.project_point(m, pt, false), FeatureId::Face(0))
    }
}
