use num::Float;
use na::Translate;
use na;
use point::PointQuery;
use entities::shape::Ball;
use math::{Point, Vector};

impl<P, M> PointQuery<P, M> for Ball<<P::Vect as Vector>::Scalar>
    where P: Point,
          M: Translate<P> {
    #[inline]
    fn project_point(&self, m: &M, pt: &P, solid: bool) -> P {
        let ls_pt  = m.inverse_translate(pt);
        let distance_squared = na::norm_squared(ls_pt.as_vector());

        if distance_squared <= self.radius() * self.radius() && solid {
            pt.clone()
        }
        else {
            let ls_proj = na::origin::<P>() + *ls_pt.as_vector() / distance_squared.sqrt();

            m.translate(&ls_proj)
        }
    }

    #[inline]
    fn distance_to_point(&self, m: &M, pt: &P, solid: bool) -> <P::Vect as Vector>::Scalar {
        let dist = na::norm(m.inverse_translate(pt).as_vector()) - self.radius();

        if dist < na::zero() {
            if solid {
                na::zero()
            }
            else {
                -dist
            }
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
