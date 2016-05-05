use na::{self, Transform};
use shape::Segment;
use query::{PointQuery, PointProjection};
use math::Point;


impl<P, M> PointQuery<P, M> for Segment<P>
    where P: Point,
          M: Transform<P> {
    #[inline]
    fn project_point(&self, m: &M, pt: &P, _: bool) -> PointProjection<P> {
        let ls_pt = m.inverse_transform(pt);
        let ab    = *self.b() - *self.a();
        let ap    = ls_pt - *self.a();
        let ab_ap = na::dot(&ab, &ap);
        let sqnab = na::norm_squared(&ab);

        let proj;

        if ab_ap <= na::zero() {
            // Voronoï region of vertex 'a'.
            proj = m.transform(self.a());
        }
        else if ab_ap >= sqnab {
            // Voronoï region of vertex 'b'.
            proj = m.transform(self.b());
        }
        else {
            assert!(sqnab != na::zero());

            // Voronoï region of the segment interior.
            proj = m.transform(&(*self.a() + ab * (ab_ap / sqnab)));
        }

        // FIXME: is this acceptable?
        let inside = na::approx_eq(&proj, pt);

        PointProjection::new(inside, proj)
    }

    // NOTE: the default implementation of `.distance_to_point(...)` will return the error that was
    // eaten by the `::approx_eq(...)` on `project_point(...)`.
}
