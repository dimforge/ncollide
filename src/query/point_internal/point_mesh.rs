use alga::general::Id;
use na;
use query::{PointProjection, PointQuery, PointQueryWithLocation};
use shape::{BaseMesh, BaseMeshElement, FeatureId, Polyline, Segment, TriMesh, Triangle};
use bounding_volume::AABB;
use partitioning::{BVTCostFn, BVTVisitor};
use math::{Isometry, Point};

impl<P, I, E> BaseMesh<P, I, E>
where
    N: Real,
    E: BaseMeshElement<I, P> + PointQuery<P, Id> + PointQueryWithLocation<P, Id>,
{
    #[inline]
    fn project_point<M: Isometry<P>>(&self, m: &Isometry<N>, point: &P, solid: bool) -> PointProjection<N> {
        let (projection, _) = self.project_point_with_location(m, point, solid);
        projection
    }

    #[inline]
    fn contains_point<M: Isometry<P>>(&self, m: &Isometry<N>, point: &P) -> bool {
        let ls_pt = m.inverse_transform_point(point);
        let mut test = PointContainementTest {
            mesh: self,
            point: &ls_pt,
            found: false,
        };

        self.bvt().visit(&mut test);

        test.found
    }

    #[inline]
    fn project_point_with_location<M: Isometry<P>>(
        &self,
        m: &Isometry<N>,
        point: &P,
        _: bool,
    ) -> (PointProjection<N>, PointProjectionInfo<E::Location>) {
        let ls_pt = m.inverse_transform_point(point);
        let mut cost_fn = BaseMeshPointProjCostFn {
            mesh: self,
            point: &ls_pt,
        };

        let (mut proj, extra_info) = self.bvt().best_first_search(&mut cost_fn).unwrap().1;
        proj.point = m.transform_point(&proj.point);

        (proj, extra_info)
    }
}

/*
 * Costs function.
 */
struct BaseMeshPointProjCostFn<'a, P: 'a + Point, I: 'a, E: 'a> {
    mesh: &'a BaseMesh<P, I, E>,
    point: &'a P,
}

impl<'a, P, I, E> BVTCostFn<N, usize, AABB<N>> for BaseMeshPointProjCostFn<'a, P, I, E>
where
    N: Real,
    E: BaseMeshElement<I, P> + PointQueryWithLocation<P, Id>,
{
    type UserData = (PointProjection<N>, PointProjectionInfo<E::Location>);

    #[inline]
    fn compute_bv_cost(&mut self, aabb: &AABB<N>) -> Option<N> {
        Some(aabb.distance_to_point(&Isometry::identity(), self.point, true))
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &usize) -> Option<(N, Self::UserData)> {
        let (proj, extra_info) =
            self.mesh
                .element_at(*b)
                .project_point_with_location(&Isometry::identity(), self.point, true);

        let extra_info = PointProjectionInfo {
            element_index: *b,
            location: extra_info,
        };

        Some((na::distance(self.point, &proj.point), (proj, extra_info)))
    }
}

/*
 * Visitor.
 */
/// Bounding Volume Tree visitor collecting nodes that may contain a given point.
struct PointContainementTest<'a, P: 'a + Point, I: 'a, E: 'a> {
    mesh: &'a BaseMesh<P, I, E>,
    point: &'a P,
    found: bool,
}

impl<'a, P, I, E> BVTVisitor<usize, AABB<N>> for PointContainementTest<'a, P, I, E>
where
    N: Real,
    E: BaseMeshElement<I, P> + PointQuery<P, Id>,
{
    #[inline]
    fn visit_internal(&mut self, bv: &AABB<N>) -> bool {
        !self.found && bv.contains_point(&Isometry::identity(), self.point)
    }

    #[inline]
    fn visit_leaf(&mut self, b: &usize, bv: &AABB<N>) {
        if !self.found && bv.contains_point(&Isometry::identity(), self.point)
            && self.mesh
                .element_at(*b)
                .contains_point(&Isometry::identity(), self.point)
        {
            self.found = true;
        }
    }
}

/// Additional point pojection info for base meshes
#[derive(Debug)]
pub struct PointProjectionInfo<C> {
    /// The index of the base mesh element the point was projected on
    ///
    /// The terminology is a bit confusing here, as this is not the index of a
    /// base mesh vertex, but rather of a base mesh element. Meaning, it is
    /// intended to be passed to `BaseMesh::element_at`.
    pub element_index: usize,

    /// Locatino of the point on the `element_index`-th element.
    ///
    /// The type of this field depends on the type of the base mesh element.
    pub location: C,
}

/*
 * fwd impls to exact meshes.
 */
impl<N: Real> PointQuery<N> for TriMesh<P> {
    #[inline]
    fn project_point(&self, m: &Isometry<N>, point: &P, solid: bool) -> PointProjection<N> {
        self.base_mesh().project_point(m, point, solid)
    }

    #[inline]
    fn project_point_with_feature(&self, m: &Isometry<N>, point: &P) -> (PointProjection<N>, FeatureId) {
        unimplemented!()
    }

    // FIXME: implement distance_to_point too?

    #[inline]
    fn contains_point(&self, m: &Isometry<N>, point: &P) -> bool {
        self.base_mesh().contains_point(m, point)
    }
}

impl<N: Real> PointQuery<N> for Polyline<P> {
    #[inline]
    fn project_point(&self, m: &Isometry<N>, point: &P, solid: bool) -> PointProjection<N> {
        let (projection, _) = self.project_point_with_location(m, point, solid);
        projection
    }

    #[inline]
    fn project_point_with_feature(&self, m: &Isometry<N>, point: &P) -> (PointProjection<N>, FeatureId) {
        unimplemented!()
    }

    // FIXME: implement distance_to_point too?

    #[inline]
    fn contains_point(&self, m: &Isometry<N>, point: &P) -> bool {
        self.base_mesh().contains_point(m, point)
    }
}

impl<N: Real> PointQueryWithLocation<N> for Polyline<P> {
    type Location = PointProjectionInfo<<Segment<N> as PointQueryWithLocation<N>>::Location>;

    #[inline]
    fn project_point_with_location(
        &self,
        m: &Isometry<N>,
        point: &P,
        solid: bool,
    ) -> (PointProjection<N>, Self::Location) {
        self.base_mesh()
            .project_point_with_location(m, point, solid)
    }
}

impl<N: Real> PointQueryWithLocation<N> for TriMesh<P> {
    type Location = PointProjectionInfo<<Triangle<N> as PointQueryWithLocation<N>>::Location>;

    #[inline]
    fn project_point_with_location(
        &self,
        m: &Isometry<N>,
        point: &P,
        solid: bool,
    ) -> (PointProjection<N>, Self::Location) {
        self.base_mesh()
            .project_point_with_location(m, point, solid)
    }
}
