use na::Transform;
use na;
use entities::shape::Segment;
use point::{LocalPointQuery, PointQuery};
use math::{Scalar, Point, Vect};


impl<P> LocalPointQuery<P> for Segment<P>
    where P: Point {
    #[inline]
    fn project_point(&self, pt: &P, _: bool) -> P {
        let ab = *self.b() - *self.a();
        let ap = *pt - *self.a();
        let ab_ap = na::dot(&ab, &ap);
        let sqnab = na::sqnorm(&ab);

        if ab_ap <= na::zero() {
            // voronoï region of vertex 'a'.
            self.a().clone()
        }
        else if ab_ap >= sqnab {
            // voronoï region of vertex 'b'.
            self.b().clone()
        }
        else {
            assert!(sqnab != na::zero());
            // voronoï region of the segment interior.
            *self.a() + ab * (ab_ap / sqnab)
        }
    }

    #[inline]
    fn distance_to_point(&self, pt: &P) -> <P::Vect as Vect>::Scalar {
        na::dist(pt, &self.project_point(pt, true))
    }

    #[inline]
    fn contains_point(&self, pt: &P) -> bool {
        na::approx_eq(&self.distance_to_point(pt), &na::zero())
    }
}

impl<P, M> PointQuery<P, M> for Segment<P>
    where P: Point,
          M: Transform<P> {
}
