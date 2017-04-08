use alga::general::Id;
use na;
use query::{AdvancedPointQuery, PointQuery, PointProjection};
use shape::{BaseMesh, BaseMeshElement, TriMesh, Polyline};
use bounding_volume::AABB;
use partitioning::{BVTCostFn, BVTVisitor};
use math::{Point, Isometry};


impl<P, M, I, E> PointQuery<P, M> for BaseMesh<P, I, E>
    where P: Point,
          M: Isometry<P>,
          E: BaseMeshElement<I, P> + PointQuery<P, Id> + AdvancedPointQuery<P, Id> {
    #[inline]
    fn project_point(&self, m: &M, point: &P, solid: bool) -> PointProjection<P> {
        self.project_point_with_info(m, point, solid).without_info()
    }

    #[inline]
    fn contains_point(&self, m: &M, point: &P) -> bool {
        let ls_pt = m.inverse_transform_point(point);
        let mut test = PointContainementTest { mesh: self, point: &ls_pt, found: false };

        self.bvt().visit(&mut test);

        test.found
    }
}

impl<P, M, I, E> AdvancedPointQuery<P, M> for BaseMesh<P, I, E>
    where P: Point,
          M: Isometry<P>,
          E: BaseMeshElement<I, P> + AdvancedPointQuery<P, Id>
{
    type ProjectionInfo = PointProjectionInfo<E::ProjectionInfo>;

    #[inline]
    fn project_point_with_info(&self, m: &M, point: &P, _: bool)
        -> PointProjection<P, Self::ProjectionInfo>
    {
        let ls_pt = m.inverse_transform_point(point);
        let mut cost_fn = BaseMeshPointProjCostFn { mesh: self, point: &ls_pt };

        let mut proj = self.bvt().best_first_search(&mut cost_fn).unwrap().1;
        proj.point = m.transform_point(&proj.point);

        proj
    }
}


/*
 * Costs function.
 */
struct BaseMeshPointProjCostFn<'a, P: 'a + Point, I: 'a, E: 'a> {
    mesh:  &'a BaseMesh<P, I, E>,
    point: &'a P
}

impl<'a, P, I, E> BVTCostFn<P::Real, usize, AABB<P>> for BaseMeshPointProjCostFn<'a, P, I, E>
    where P: Point,
          E: BaseMeshElement<I, P> + AdvancedPointQuery<P, Id> {
    type UserData = PointProjection<P, PointProjectionInfo<E::ProjectionInfo>>;

    #[inline]
    fn compute_bv_cost(&mut self, aabb: &AABB<P>) -> Option<P::Real> {
        Some(aabb.distance_to_point(&Id::new(), self.point, true))
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &usize) -> Option<(P::Real, Self::UserData)> {
        let proj = self.mesh
            .element_at(*b)
            .project_point_with_info(&Id::new(), self.point, true);

        let proj = PointProjection {
            is_inside: proj.is_inside,
            point    : proj.point,

            info: PointProjectionInfo {
                element_index      : *b,
                position_on_element: proj.info,
            },
        };

        Some((na::distance(self.point, &proj.point), proj))
    }
}

/*
 * Visitor.
 */
/// Bounding Volume Tree visitor collecting nodes that may contain a given point.
struct PointContainementTest<'a, P: 'a + Point, I: 'a, E: 'a> {
    mesh:  &'a BaseMesh<P, I, E>,
    point: &'a P,
    found: bool
}

impl<'a, P, I, E> BVTVisitor<usize, AABB<P>> for PointContainementTest<'a, P, I, E>
    where P: Point,
          E: BaseMeshElement<I, P> + PointQuery<P, Id> {
    #[inline]
    fn visit_internal(&mut self, bv: &AABB<P>) -> bool {
        !self.found && bv.contains_point(&Id::new(), self.point)
    }

    #[inline]
    fn visit_leaf(&mut self, b: &usize, bv: &AABB<P>) {
        if !self.found &&
           bv.contains_point(&Id::new(), self.point) &&
           self.mesh.element_at(*b).contains_point(&Id::new(), self.point) {
            self.found = true;
        }
    }
}


/// Additional point pojection info for base meshes
pub struct PointProjectionInfo<P> {
    /// The index of the base mesh element the point was projected on
    ///
    /// The terminology is a bit confusing here, as this is not the index of a
    /// base mesh vertex, but rather of a base mesh element. Meaning, it is
    /// intended to be passed to `BaseMesh::element_at`.
    pub element_index: usize,

    /// Describes where on the base mesh element the point was projected
    ///
    /// The type of this field depends on the type of the base mesh element.
    pub position_on_element: P,
}


/*
 * fwd impls to exact meshes.
 */
impl<P: Point, M: Isometry<P>> PointQuery<P, M> for TriMesh<P> {
    #[inline]
    fn project_point(&self, m: &M, point: &P, solid: bool) -> PointProjection<P> {
        self.base_mesh().project_point(m, point, solid)
    }

    #[inline]
    fn distance_to_point(&self, m: &M, point: &P, solid: bool) -> P::Real {
        self.base_mesh().distance_to_point(m, point, solid)
    }

    #[inline]
    fn contains_point(&self, m: &M, point: &P) -> bool {
        self.base_mesh().contains_point(m, point)
    }
}

impl<P: Point, M: Isometry<P>> PointQuery<P, M> for Polyline<P> {
    #[inline]
    fn project_point(&self, m: &M, point: &P, solid: bool) -> PointProjection<P> {
        self.base_mesh().project_point(m, point, solid)
    }

    #[inline]
    fn distance_to_point(&self, m: &M, point: &P, solid: bool) -> P::Real {
        self.base_mesh().distance_to_point(m, point, solid)
    }

    #[inline]
    fn contains_point(&self, m: &M, point: &P) -> bool {
        self.base_mesh().contains_point(m, point)
    }
}
