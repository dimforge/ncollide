use na::Transform;
use na;
use entities::shape::Segment;
use point::PointQuery;
use math::{Point, Vector};


impl<P, M> PointQuery<P, M> for Segment<P>
    where P: Point,
          M: Transform<P> {
    #[inline]
    fn project_point(&self, m: &M, pt: &P, _: bool) -> P {
        let ls_pt = m.inverse_transform(pt);
        let ab    = *self.b() - *self.a();
        let ap    = ls_pt - *self.a();
        let ab_ap = na::dot(&ab, &ap);
        let sqnab = na::norm_squared(&ab);

        if ab_ap <= na::zero() {
            // voronoï region of vertex 'a'.
            m.transform(self.a())
        }
        else if ab_ap >= sqnab {
            // voronoï region of vertex 'b'.
            m.transform(self.b())
        }
        else {
            assert!(sqnab != na::zero());
            // voronoï region of the segment interior.
            m.transform(&(*self.a() + ab * (ab_ap / sqnab)))
        }
    }

    #[inline]
    fn distance_to_point(&self, m: &M, pt: &P) -> <P::Vect as Vector>::Scalar {
        na::distance(pt, &self.project_point(m, pt, true))
    }

    #[inline]
    fn contains_point(&self, m: &M, pt: &P) -> bool {
        na::approx_eq(&self.distance_to_point(m, pt), &na::zero())
    }
}
