use num::Float;
use na::{self, Translate};
use query::{PointQuery, PointProjection};
use shape::Ball;
use math::{Point, Vector};

impl<P, M> PointQuery<P, M> for Ball<<P::Vect as Vector>::Scalar>
    where P: Point,
          M: Translate<P> {
    #[inline]
    fn project_point(&self, m: &M, pt: &P, solid: bool) -> PointProjection<P> {
        let ls_pt  = m.inverse_translate(pt);
        let distance_squared = na::norm_squared(ls_pt.as_vector());

        let inside = distance_squared <= self.radius() * self.radius();

        if inside && solid {
            PointProjection::new(true, pt.clone())
        }
        else {
            let ls_proj = na::origin::<P>() + *ls_pt.as_vector() / distance_squared.sqrt();

            PointProjection::new(inside, m.translate(&ls_proj))
        }
    }

    #[inline]
    fn distance_to_point(&self, m: &M, pt: &P, solid: bool) -> <P::Vect as Vector>::Scalar {
        let dist = na::norm(m.inverse_translate(pt).as_vector()) - self.radius();

        if solid && dist < na::zero() {
            na::zero()
        }
        else {
            dist
        }
    }

    #[inline]
    fn contains_point(&self, m: &M, pt: &P) -> bool {
        na::norm_squared(m.inverse_translate(pt).as_vector()) <= self.radius() * self.radius()
    }
}
