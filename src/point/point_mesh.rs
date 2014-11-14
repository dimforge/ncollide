use na::Transform;
use na;
use point::{LocalPointQuery, PointQuery};
use shape::{Mesh, MeshElement};
use bounding_volume::AABB;
use partitioning::{BVTCostFn, BVTVisitor};
use math::{Scalar, Point, Vect};


impl<N, P, V, E> LocalPointQuery<N, P> for Mesh<N, P, V, E>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          E: MeshElement<P> + LocalPointQuery<N, P> {
    #[inline]
    fn project_point(&self, point: &P, _: bool) -> P {
        let mut cost_fn = MeshPointProjCostFn { mesh: self, point: point };

        self.bvt().best_first_search(&mut cost_fn).unwrap().val1()
    }

    #[inline]
    fn distance_to_point(&self, point: &P) -> N {
        na::dist(point, &self.project_point(point, true))
    }

    #[inline]
    fn contains_point(&self, point: &P) -> bool {
        let mut test = PointContainementTest { mesh: self, point: point, found: false };

        self.bvt().visit(&mut test);

        test.found
    }
}

impl<N, P, V, E, M> PointQuery<N, P, M> for Mesh<N, P, V, E>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P>,
          E: MeshElement<P> + LocalPointQuery<N, P> {
}


/*
 * Costs function.
 */
struct MeshPointProjCostFn<'a, N: 'a, P: 'a, V: 'a, E: 'a> {
    mesh:  &'a Mesh<N, P, V, E>,
    point: &'a P
}

impl<'a, N, P, V, E> BVTCostFn<N, uint, AABB<P>, P> for MeshPointProjCostFn<'a, N, P, V, E>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          E: MeshElement<P> + LocalPointQuery<N, P> {
    #[inline]
    fn compute_bv_cost(&mut self, aabb: &AABB<P>) -> Option<N> {
        Some(aabb.distance_to_point(self.point))
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &uint) -> Option<(N, P)> {
        let proj = self.mesh.element_at(*b).project_point(self.point, true);
        
        Some((na::dist(self.point, &proj), proj))
    }
}

/*
 * Visitor.
 */
/// Bounding Volume Tree visitor collecting nodes that may contain a given point.
struct PointContainementTest<'a, N: 'a, P: 'a, V: 'a, E: 'a> {
    mesh:  &'a Mesh<N, P, V, E>,
    point: &'a P,
    found: bool
}

impl<'a, N, P, V, E> BVTVisitor<uint, AABB<P>> for PointContainementTest<'a, N, P, V, E>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          E: MeshElement<P> + LocalPointQuery<N, P> {
    #[inline]
    fn visit_internal(&mut self, bv: &AABB<P>) -> bool {
        !self.found && bv.contains_point(self.point)
    }

    #[inline]
    fn visit_leaf(&mut self, b: &uint, bv: &AABB<P>) {
        if !self.found &&
           bv.contains_point(self.point) &&
           self.mesh.element_at(*b).contains_point(self.point) {
            self.found = true;
        }
    }
}
