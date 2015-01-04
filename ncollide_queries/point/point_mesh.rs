use na::Transform;
use na;
use point::{LocalPointQuery, PointQuery};
use entities::shape::{BaseMesh, BaseMeshElement, TriMesh, Polyline};
use entities::bounding_volume::AABB;
use entities::partitioning::{BVTCostFn, BVTVisitor};
use math::{Scalar, Point, Vect};


impl<N, P, V, I, E> LocalPointQuery<N, P> for BaseMesh<N, P, V, I, E>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          E: BaseMeshElement<I, P> + LocalPointQuery<N, P> {
    #[inline]
    fn project_point(&self, point: &P, _: bool) -> P {
        let mut cost_fn = BaseMeshPointProjCostFn { mesh: self, point: point };

        self.bvt().best_first_search(&mut cost_fn).unwrap().1
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

impl<N, P, V, M, I, E> PointQuery<N, P, M> for BaseMesh<N, P, V, I, E>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P>,
          E: BaseMeshElement<I, P> + LocalPointQuery<N, P> {
}


/*
 * Costs function.
 */
struct BaseMeshPointProjCostFn<'a, N: 'a, P: 'a, V: 'a, I: 'a, E: 'a> {
    mesh:  &'a BaseMesh<N, P, V, I, E>,
    point: &'a P
}

impl<'a, N, P, V, I, E> BVTCostFn<N, uint, AABB<P>, P> for BaseMeshPointProjCostFn<'a, N, P, V, I, E>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          E: BaseMeshElement<I, P> + LocalPointQuery<N, P> {
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
struct PointContainementTest<'a, N: 'a, P: 'a, V: 'a, I: 'a, E: 'a> {
    mesh:  &'a BaseMesh<N, P, V, I, E>,
    point: &'a P,
    found: bool
}

impl<'a, N, P, V, I, E> BVTVisitor<uint, AABB<P>> for PointContainementTest<'a, N, P, V, I, E>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          E: BaseMeshElement<I, P> + LocalPointQuery<N, P> {
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

/*
 * fwd impls to exact meshes.
 */
impl<N, P, V> LocalPointQuery<N, P> for TriMesh<N, P, V>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    #[inline]
    fn project_point(&self, point: &P, solid: bool) -> P {
        self.base_mesh().project_point(point, solid)
    }

    #[inline]
    fn distance_to_point(&self, point: &P) -> N {
        self.base_mesh().distance_to_point(point)
    }

    #[inline]
    fn contains_point(&self, point: &P) -> bool {
        self.base_mesh().contains_point(point)
    }
}

impl<N, P, V, M> PointQuery<N, P, M> for TriMesh<N, P, V>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> {
}

impl<N, P, V> LocalPointQuery<N, P> for Polyline<N, P, V>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    #[inline]
    fn project_point(&self, point: &P, solid: bool) -> P {
        self.base_mesh().project_point(point, solid)
    }

    #[inline]
    fn distance_to_point(&self, point: &P) -> N {
        self.base_mesh().distance_to_point(point)
    }

    #[inline]
    fn contains_point(&self, point: &P) -> bool {
        self.base_mesh().contains_point(point)
    }
}

impl<N, P, V, M> PointQuery<N, P, M> for Polyline<N, P, V>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> {
}
