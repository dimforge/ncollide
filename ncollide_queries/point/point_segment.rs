use na::Transform;
use na;
use entities::shape::Segment;
use point::{LocalPointQuery, PointQuery};
use math::{Scalar, Point, Vect};


#[old_impl_check]
impl<N, P, V> LocalPointQuery<N, P> for Segment<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
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
    fn distance_to_point(&self, pt: &P) -> N {
        na::dist(pt, &self.project_point(pt, true))
    }

    #[inline]
    fn contains_point(&self, pt: &P) -> bool {
        na::approx_eq(&self.distance_to_point(pt), &na::zero())
    }
}

#[old_impl_check]
impl<N, P, V, M> PointQuery<N, P, M> for Segment<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> {
}
