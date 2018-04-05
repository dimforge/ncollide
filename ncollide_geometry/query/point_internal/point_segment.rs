use na;
use shape::{FeatureId, Segment, SegmentPointLocation};
use query::{PointProjection, PointQuery, PointQueryWithLocation};
use math::{Isometry, Point};

impl<P: Point, M: Isometry<P>> PointQuery<P, M> for Segment<P> {
    #[inline]
    fn project_point(&self, m: &M, pt: &P, solid: bool) -> PointProjection<P> {
        let (projection, _) = self.project_point_with_location(m, pt, solid);
        projection
    }

    #[inline]
    fn project_point_with_feature(&self, m: &M, pt: &P) -> (PointProjection<P>, FeatureId) {
        let (proj, loc) = self.project_point_with_location(m, pt, false);
        let feature = match loc {
            SegmentPointLocation::OnVertex(i) => FeatureId::Vertex(i),
            SegmentPointLocation::OnEdge(..) => FeatureId::Edge(0),
        };

        (proj, feature)
    }

    // NOTE: the default implementation of `.distance_to_point(...)` will return the error that was
    // eaten by the `::approx_eq(...)` on `project_point(...)`.
}

impl<P: Point, M: Isometry<P>> PointQueryWithLocation<P, M> for Segment<P> {
    type Location = SegmentPointLocation<P::Real>;

    #[inline]
    fn project_point_with_location(
        &self,
        m: &M,
        pt: &P,
        _: bool,
    ) -> (PointProjection<P>, Self::Location) {
        let ls_pt = m.inverse_transform_point(pt);
        let ab = *self.b() - *self.a();
        let ap = ls_pt - *self.a();
        let ab_ap = na::dot(&ab, &ap);
        let sqnab = na::norm_squared(&ab);
        let _1 = na::one::<P::Real>();

        let mut proj;
        let location;

        if ab_ap <= na::zero() {
            // Voronoï region of vertex 'a'.
            location = SegmentPointLocation::OnVertex(0);
            proj = m.transform_point(self.a());
        } else if ab_ap >= sqnab {
            // Voronoï region of vertex 'b'.
            location = SegmentPointLocation::OnVertex(1);
            proj = m.transform_point(self.b());
        } else {
            assert!(sqnab != na::zero());

            // Voronoï region of the segment interior.
            let u = ab_ap / sqnab;
            let bcoords = [_1 - u, u];
            location = SegmentPointLocation::OnEdge(bcoords);
            proj = *self.a();
            proj.axpy(bcoords[1], self.b(), bcoords[0]);
            proj = m.transform_point(&proj);
        }

        // FIXME: is this acceptable?
        let inside = relative_eq!(proj, *pt);

        (PointProjection::new(inside, proj), location)
    }
}
