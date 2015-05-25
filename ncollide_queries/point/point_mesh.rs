use na::Transform;
use na;
use point::{LocalPointQuery, PointQuery};
use entities::shape::{BaseMesh, BaseMeshElement, TriMesh, Polyline};
use entities::bounding_volume::AABB;
use entities::partitioning::{BVTCostFn, BVTVisitor};
use math::{Scalar, Point, Vect};


impl<P, I, E> LocalPointQuery<P> for BaseMesh<P, I, E>
    where P: Point,
          E: BaseMeshElement<I, P> + LocalPointQuery<P> {
    #[inline]
    fn project_point(&self, point: &P, _: bool) -> P {
        let mut cost_fn = BaseMeshPointProjCostFn { mesh: self, point: point };

        self.bvt().best_first_search(&mut cost_fn).unwrap().1
    }

    #[inline]
    fn distance_to_point(&self, point: &P) -> <P::Vect as Vect>::Scalar {
        na::dist(point, &self.project_point(point, true))
    }

    #[inline]
    fn contains_point(&self, point: &P) -> bool {
        let mut test = PointContainementTest { mesh: self, point: point, found: false };

        self.bvt().visit(&mut test);

        test.found
    }
}

impl<P, M, I, E> PointQuery<P, M> for BaseMesh<P, I, E>
    where P: Point,
          M: Transform<P>,
          E: BaseMeshElement<I, P> + LocalPointQuery<P> {
}


/*
 * Costs function.
 */
struct BaseMeshPointProjCostFn<'a, P: 'a + Point, I: 'a, E: 'a> {
    mesh:  &'a BaseMesh<P, I, E>,
    point: &'a P
}

impl<'a, P, I, E> BVTCostFn<<P::Vect as Vect>::Scalar, usize, AABB<P>, P> for BaseMeshPointProjCostFn<'a, P, I, E>
    where P: Point,
          E: BaseMeshElement<I, P> + LocalPointQuery<P> {
    #[inline]
    fn compute_bv_cost(&mut self, aabb: &AABB<P>) -> Option<<P::Vect as Vect>::Scalar> {
        Some(aabb.distance_to_point(self.point))
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &usize) -> Option<(<P::Vect as Vect>::Scalar, P)> {
        let proj = self.mesh.element_at(*b).project_point(self.point, true);
        
        Some((na::dist(self.point, &proj), proj))
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
          E: BaseMeshElement<I, P> + LocalPointQuery<P> {
    #[inline]
    fn visit_internal(&mut self, bv: &AABB<P>) -> bool {
        !self.found && bv.contains_point(self.point)
    }

    #[inline]
    fn visit_leaf(&mut self, b: &usize, bv: &AABB<P>) {
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
impl<P> LocalPointQuery<P> for TriMesh<P>
    where P: Point {
    #[inline]
    fn project_point(&self, point: &P, solid: bool) -> P {
        self.base_mesh().project_point(point, solid)
    }

    #[inline]
    fn distance_to_point(&self, point: &P) -> <P::Vect as Vect>::Scalar {
        self.base_mesh().distance_to_point(point)
    }

    #[inline]
    fn contains_point(&self, point: &P) -> bool {
        self.base_mesh().contains_point(point)
    }
}

impl<P, M> PointQuery<P, M> for TriMesh<P>
    where P: Point,
          M: Transform<P> {
}

impl<P> LocalPointQuery<P> for Polyline<P>
    where P: Point {
    #[inline]
    fn project_point(&self, point: &P, solid: bool) -> P {
        self.base_mesh().project_point(point, solid)
    }

    #[inline]
    fn distance_to_point(&self, point: &P) -> <P::Vect as Vect>::Scalar {
        self.base_mesh().distance_to_point(point)
    }

    #[inline]
    fn contains_point(&self, point: &P) -> bool {
        self.base_mesh().contains_point(point)
    }
}

impl<P, M> PointQuery<P, M> for Polyline<P>
    where P: Point,
          M: Transform<P> {
}
