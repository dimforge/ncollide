use na::{Transform, Identity};
use na;
use point::PointQuery;
use entities::shape::{BaseMesh, BaseMeshElement, TriMesh, Polyline};
use entities::bounding_volume::AABB;
use entities::partitioning::{BVTCostFn, BVTVisitor};
use math::{Point, Vector};


impl<P, M, I, E> PointQuery<P, M> for BaseMesh<P, I, E>
    where P: Point,
          M: Transform<P>,
          E: BaseMeshElement<I, P> + PointQuery<P, Identity> {
    #[inline]
    fn project_point(&self, m: &M, point: &P, _: bool) -> P {
        let ls_pt = m.inverse_transform(point);
        let mut cost_fn = BaseMeshPointProjCostFn { mesh: self, point: &ls_pt };

        m.transform(&self.bvt().best_first_search(&mut cost_fn).unwrap().1)
    }

    #[inline]
    fn distance_to_point(&self, m: &M, point: &P) -> <P::Vect as Vector>::Scalar {
        na::distance(point, &self.project_point(m, point, true))
    }

    #[inline]
    fn contains_point(&self, m: &M, point: &P) -> bool {
        let ls_pt = m.inverse_transform(point);
        let mut test = PointContainementTest { mesh: self, point: &ls_pt, found: false };

        self.bvt().visit(&mut test);

        test.found
    }
}


/*
 * Costs function.
 */
struct BaseMeshPointProjCostFn<'a, P: 'a + Point, I: 'a, E: 'a> {
    mesh:  &'a BaseMesh<P, I, E>,
    point: &'a P
}

impl<'a, P, I, E> BVTCostFn<<P::Vect as Vector>::Scalar, usize, AABB<P>> for BaseMeshPointProjCostFn<'a, P, I, E>
    where P: Point,
          E: BaseMeshElement<I, P> + PointQuery<P, Identity> {
    type UserData = P;

    #[inline]
    fn compute_bv_cost(&mut self, aabb: &AABB<P>) -> Option<<P::Vect as Vector>::Scalar> {
        Some(aabb.distance_to_point(&Identity::new(), self.point))
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &usize) -> Option<(<P::Vect as Vector>::Scalar, P)> {
        let proj = self.mesh.element_at(*b).project_point(&Identity::new(), self.point, true);

        Some((na::distance(self.point, &proj), proj))
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
          E: BaseMeshElement<I, P> + PointQuery<P, Identity> {
    #[inline]
    fn visit_internal(&mut self, bv: &AABB<P>) -> bool {
        !self.found && bv.contains_point(&Identity::new(), self.point)
    }

    #[inline]
    fn visit_leaf(&mut self, b: &usize, bv: &AABB<P>) {
        if !self.found &&
           bv.contains_point(&Identity::new(), self.point) &&
           self.mesh.element_at(*b).contains_point(&Identity::new(), self.point) {
            self.found = true;
        }
    }
}

/*
 * fwd impls to exact meshes.
 */
impl<P, M> PointQuery<P, M> for TriMesh<P>
    where P: Point,
          M: Transform<P> {
    #[inline]
    fn project_point(&self, m: &M, point: &P, solid: bool) -> P {
        self.base_mesh().project_point(m, point, solid)
    }

    #[inline]
    fn distance_to_point(&self, m: &M, point: &P) -> <P::Vect as Vector>::Scalar {
        self.base_mesh().distance_to_point(m, point)
    }

    #[inline]
    fn contains_point(&self, m: &M, point: &P) -> bool {
        self.base_mesh().contains_point(m, point)
    }
}

impl<P, M> PointQuery<P, M> for Polyline<P>
    where P: Point,
          M: Transform<P> {
    #[inline]
    fn project_point(&self, m: &M, point: &P, solid: bool) -> P {
        self.base_mesh().project_point(m, point, solid)
    }

    #[inline]
    fn distance_to_point(&self, m: &M, point: &P) -> <P::Vect as Vector>::Scalar {
        self.base_mesh().distance_to_point(m, point)
    }

    #[inline]
    fn contains_point(&self, m: &M, point: &P) -> bool {
        self.base_mesh().contains_point(m, point)
    }
}
