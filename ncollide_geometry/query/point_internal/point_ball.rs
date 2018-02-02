use alga::general::Real;
use na;

use query::{PointProjection, PointQuery};
use shape::Ball;
use math::{Isometry, Point};

impl<P: Point, M: Isometry<P>> PointQuery<P, M> for Ball<P::Real> {
    #[inline]
    fn project_point(&self, m: &M, pt: &P, solid: bool) -> PointProjection<P> {
        let ls_pt = m.inverse_translate_point(pt);
        let distance_squared = na::norm_squared(&ls_pt.coordinates());

        let inside = distance_squared <= self.radius() * self.radius();

        if inside && solid {
            PointProjection::new(true, *pt)
        } else {
            let ls_proj = P::origin() + ls_pt.coordinates() / distance_squared.sqrt();

            PointProjection::new(inside, m.translate_point(&ls_proj))
        }
    }

    #[inline]
    fn distance_to_point(&self, m: &M, pt: &P, solid: bool) -> P::Real {
        let dist = na::norm(&m.inverse_translate_point(pt).coordinates()) - self.radius();

        if solid && dist < na::zero() {
            na::zero()
        } else {
            dist
        }
    }

    #[inline]
    fn contains_point(&self, m: &M, pt: &P) -> bool {
        na::norm_squared(&m.inverse_translate_point(pt).coordinates())
            <= self.radius() * self.radius()
    }
}
