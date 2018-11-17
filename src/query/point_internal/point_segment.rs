use math::{Isometry, Point};
use na::{self, Real};
use query::{PointProjection, PointQuery, PointQueryWithLocation};
use shape::{FeatureId, Segment, SegmentPointLocation};
use utils::IsometryOps;

impl<N: Real> PointQuery<N> for Segment<N> {
    #[inline]
    fn project_point(&self, m: &Isometry<N>, pt: &Point<N>, solid: bool) -> PointProjection<N> {
        let (projection, _) = self.project_point_with_location(m, pt, solid);
        projection
    }

    #[inline]
    fn project_point_with_feature(
        &self,
        m: &Isometry<N>,
        pt: &Point<N>,
    ) -> (PointProjection<N>, FeatureId)
    {
        let (proj, loc) = self.project_point_with_location(m, pt, false);
        let feature = match loc {
            SegmentPointLocation::OnVertex(i) => FeatureId::Vertex(i),
            SegmentPointLocation::OnEdge(..) => {
                #[cfg(feature = "dim2")]
                {
                    let dir = self.scaled_direction();
                    let dpt = *pt - proj.point;
                    if dpt.perp(&dir) >= na::zero() {
                        FeatureId::Face(0)
                    } else {
                        FeatureId::Face(1)
                    }
                }

                #[cfg(feature = "dim3")]
                {
                    FeatureId::Edge(0)
                }
            }
        };

        (proj, feature)
    }

    // NOTE: the default implementation of `.distance_to_point(...)` will return the error that was
    // eaten by the `::approx_eq(...)` on `project_point(...)`.
}

impl<N: Real> PointQueryWithLocation<N> for Segment<N> {
    type Location = SegmentPointLocation<N>;

    #[inline]
    fn project_point_with_location(
        &self,
        m: &Isometry<N>,
        pt: &Point<N>,
        _: bool,
    ) -> (PointProjection<N>, Self::Location)
    {
        let ls_pt = m.inverse_transform_point(pt);
        let ab = *self.b() - *self.a();
        let ap = ls_pt - *self.a();
        let ab_ap = na::dot(&ab, &ap);
        let sqnab = na::norm_squared(&ab);
        let _1 = na::one::<N>();

        let mut proj;
        let location;

        if ab_ap <= na::zero() {
            // Voronoï region of vertex 'a'.
            location = SegmentPointLocation::OnVertex(0);
            proj = m * self.a();
        } else if ab_ap >= sqnab {
            // Voronoï region of vertex 'b'.
            location = SegmentPointLocation::OnVertex(1);
            proj = m * self.b();
        } else {
            assert!(sqnab != na::zero());

            // Voronoï region of the segment interior.
            let u = ab_ap / sqnab;
            let bcoords = [_1 - u, u];
            location = SegmentPointLocation::OnEdge(bcoords);
            proj = *self.a() + ab * u;
            proj = m * proj;
        }

        // FIXME: is this acceptable?
        let inside = relative_eq!(proj, *pt);

        (PointProjection::new(inside, proj), location)
    }
}
