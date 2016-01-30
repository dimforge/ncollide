use num::Float;
use na::Translate;
use na;
use point::PointQuery;
use entities::shape::Ball;
use math::{Point, Vect};

impl<P, M> PointQuery<P, M> for Ball<<P::Vect as Vect>::Scalar>
    where P: Point,
          M: Translate<P> {
    #[inline]
    fn project_point(&self, m: &M, pt: &P, solid: bool) -> P {
        let ls_pt  = m.inv_translate(pt);
        let sqdist = na::sqnorm(ls_pt.as_vec());

        if sqdist <= self.radius() * self.radius() && solid {
            pt.clone()
        }
        else {
            let ls_proj = na::orig::<P>() + *ls_pt.as_vec() / sqdist.sqrt();

            m.translate(&ls_proj)
        }
    }

    #[inline]
    fn distance_to_point(&self, m: &M, pt: &P) -> <P::Vect as Vect>::Scalar {
        (na::norm(m.inv_translate(pt).as_vec()) - self.radius()).max(na::zero())
    }

    #[inline]
    fn contains_point(&self, m: &M, pt: &P) -> bool {
        na::sqnorm(m.inv_translate(pt).as_vec()) <= self.radius() * self.radius()
    }
}
