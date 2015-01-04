use std::ops::Index;
use na::{Pnt2, Vec3, Transform, Rotate};
use na;
use ray::{Ray, LocalRayCast, RayCast, RayIntersection};
use ray;
use entities::shape::{BaseMesh, BaseMeshElement, TriMesh, Polyline};
use entities::bounding_volume::AABB;
use entities::partitioning::BVTCostFn;
use math::{Scalar, Point, Vect};


impl<N, P, V, I, E> LocalRayCast<N, P, V> for BaseMesh<N, P, V, I, E>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          I: Index<uint, uint>,
          E: BaseMeshElement<I, P> + LocalRayCast<N, P, V> {
    #[inline]
    fn toi_with_ray(&self, ray: &Ray<P, V>, _: bool) -> Option<N> {
        let mut cost_fn = BaseMeshRayToiCostFn { mesh: self, ray: ray };

        self.bvt().best_first_search(&mut cost_fn).map(|(_, res)| res)
    }

    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray<P, V>, _: bool) -> Option<RayIntersection<N, V>> {
        let mut cost_fn = BaseMeshRayToiAndNormalCostFn { mesh: self, ray: ray };

        self.bvt().best_first_search(&mut cost_fn).map(|(_, res)| res)
    }

    fn toi_and_normal_and_uv_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<RayIntersection<N, V>> {
        if self.uvs().is_none() || na::dim::<P>() != 3 {
            return self.toi_and_normal_with_ray(ray, solid);
        }

        let mut cost_fn = BaseMeshRayToiAndNormalAndUVsCostFn { mesh: self, ray: ray };
        let cast = self.bvt().best_first_search(&mut cost_fn);

        match cast {
            None                => None,
            Some((best, inter)) => {
                let toi = inter.0.toi;
                let n   = inter.0.normal.clone();
                let uv  = inter.1; // barycentric coordinates to compute the exact uvs.

                let idx   = &self.indices()[*best];
                let uvs   = self.uvs().as_ref().unwrap();

                let uv1 = uvs[idx[0]];
                let uv2 = uvs[idx[1]];
                let uv3 = uvs[idx[2]];

                let uvx = uv1.x * uv.x + uv2.x * uv.y + uv3.x * uv.z;
                let uvy = uv1.y * uv.x + uv2.y * uv.y + uv3.y * uv.z;

                // XXX: this interpolation should be done on the two other ray cast too!
                match *self.normals() {
                    None         => {
                        Some(RayIntersection::new_with_uvs(toi, n, Some(Pnt2::new(uvx, uvy))))
                    },
                    Some(ref ns) => {
                        let n1 = &ns[idx[0]];
                        let n2 = &ns[idx[1]];
                        let n3 = &ns[idx[2]];

                        let mut n123 = *n1 * uv.x + *n2 * uv.y + *n3 * uv.z;

                        if na::is_zero(&n123.normalize()) {
                            Some(RayIntersection::new_with_uvs(toi, n, Some(Pnt2::new(uvx, uvy))))
                        }
                        else {
                            if na::dot(&n123, &ray.dir) > na::zero() {
                                Some(RayIntersection::new_with_uvs(toi, -n123, Some(Pnt2::new(uvx, uvy))))
                            }
                            else {
                                Some(RayIntersection::new_with_uvs(toi, n123, Some(Pnt2::new(uvx, uvy))))
                            }
                        }
                    }
                }
            }
        }
    }
}

impl<N, P, V, M, I, E> RayCast<N, P, V, M> for BaseMesh<N, P, V, I, E>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> + Rotate<V>,
          I: Index<uint, uint>,
          E: BaseMeshElement<I, P> + LocalRayCast<N, P, V> {
}


/*
 * Costs functions.
 */
struct BaseMeshRayToiCostFn<'a, N: 'a, P: 'a, V: 'a, I: 'a, E: 'a> {
    mesh:  &'a BaseMesh<N, P, V, I, E>,
    ray:   &'a Ray<P, V>
}

impl<'a, N, P, V, I, E> BVTCostFn<N, uint, AABB<P>, N> for BaseMeshRayToiCostFn<'a, N, P, V, I, E>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          E: BaseMeshElement<I, P> + LocalRayCast<N, P, V> {
    #[inline]
    fn compute_bv_cost(&mut self, aabb: &AABB<P>) -> Option<N> {
        aabb.toi_with_ray(self.ray, true)
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &uint) -> Option<(N, N)> {
        self.mesh.element_at(*b).toi_with_ray(self.ray, true).map(|toi| (toi, toi))
    }
}

struct BaseMeshRayToiAndNormalCostFn<'a, N: 'a, P: 'a, V: 'a, I: 'a, E: 'a> {
    mesh:  &'a BaseMesh<N, P, V, I, E>,
    ray:   &'a Ray<P, V>
}

impl<'a, N, P, V, I, E> BVTCostFn<N, uint, AABB<P>, RayIntersection<N, V>>
for BaseMeshRayToiAndNormalCostFn<'a, N, P, V, I, E>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          E: BaseMeshElement<I, P> + LocalRayCast<N, P, V> {
    #[inline]
    fn compute_bv_cost(&mut self, aabb: &AABB<P>) -> Option<N> {
        aabb.toi_with_ray(self.ray, true)
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &uint) -> Option<(N, RayIntersection<N, V>)> {
        self.mesh.element_at(*b).toi_and_normal_with_ray(self.ray, true).map(|inter| (inter.toi, inter))
    }
}

struct BaseMeshRayToiAndNormalAndUVsCostFn<'a, N: 'a, P: 'a, V: 'a, I: 'a, E: 'a> {
    mesh:  &'a BaseMesh<N, P, V, I, E>,
    ray:   &'a Ray<P, V>
}

impl<'a, N, P, V, I, E> BVTCostFn<N, uint, AABB<P>, (RayIntersection<N, V>, Vec3<N>)>
for BaseMeshRayToiAndNormalAndUVsCostFn<'a, N, P, V, I, E>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          I: Index<uint, uint>,
          E: BaseMeshElement<I, P> + LocalRayCast<N, P, V> {
    #[inline]
    fn compute_bv_cost(&mut self, aabb: &AABB<P>) -> Option<N> {
        aabb.toi_with_ray(self.ray, true)
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &uint) -> Option<(N, (RayIntersection<N, V>, Vec3<N>))> {
        let vs = self.mesh.vertices().as_slice();
        let idx = &self.mesh.indices()[*b];

        let a = &vs[idx[0]];
        let b = &vs[idx[1]];
        let c = &vs[idx[2]];

        ray::triangle_ray_intersection(a, b, c, self.ray).map(|inter| (inter.0.toi.clone(), inter))
    }
}

/*
 * fwd impls. to the exact shapes.
 */
impl<N, P, V> LocalRayCast<N, P, V> for TriMesh<N, P, V>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    #[inline]
    fn toi_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<N> {
        self.base_mesh().toi_with_ray(ray, solid)
    }

    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<RayIntersection<N, V>> {
        self.base_mesh().toi_and_normal_with_ray(ray, solid)
    }

    #[inline]
    fn toi_and_normal_and_uv_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<RayIntersection<N, V>> {
        self.base_mesh().toi_and_normal_and_uv_with_ray(ray, solid)
    }
}

impl<N, P, V, M> RayCast<N, P, V, M> for TriMesh<N, P, V>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> + Rotate<V> {
}

impl<N, P, V> LocalRayCast<N, P, V> for Polyline<N, P, V>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    #[inline]
    fn toi_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<N> {
        self.base_mesh().toi_with_ray(ray, solid)
    }

    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<RayIntersection<N, V>> {
        self.base_mesh().toi_and_normal_with_ray(ray, solid)
    }

    #[inline]
    fn toi_and_normal_and_uv_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<RayIntersection<N, V>> {
        self.base_mesh().toi_and_normal_and_uv_with_ray(ray, solid)
    }
}

impl<N, P, V, M> RayCast<N, P, V, M> for Polyline<N, P, V>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> + Rotate<V> {
}
