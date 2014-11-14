use na::Transform;
use na;
use point::{LocalPointQuery, PointQuery};
use shape::{Mesh, MeshElement};
use bounding_volume::AABB;
use partitioning::BVTCostFn;
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
        na::approx_eq(&self.distance_to_point(point), &na::zero())
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
