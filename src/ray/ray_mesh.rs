use na::{Pnt2, Vec3, Transform, Rotate};
use na;
use ray::{Ray, LocalRayCast, RayCast, RayIntersection};
use ray;
use shape::{Mesh, MeshElement};
use bounding_volume::AABB;
use partitioning::BVTCostFn;
use math::{Scalar, Point, Vect};


impl<N, P, V, E> LocalRayCast<N, P, V> for Mesh<N, P, V, E>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          E: MeshElement<P> + LocalRayCast<N, P, V> {
    #[inline]
    fn toi_with_ray(&self, ray: &Ray<P, V>, _: bool) -> Option<N> {
        let mut cost_fn = MeshRayToiCostFn { mesh: self, ray: ray };

        self.bvt().best_first_search(&mut cost_fn).map(|(_, res)| res)
    }

    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray<P, V>, _: bool) -> Option<RayIntersection<N, V>> {
        let mut cost_fn = MeshRayToiAndNormalCostFn { mesh: self, ray: ray };

        self.bvt().best_first_search(&mut cost_fn).map(|(_, res)| res)
    }

    fn toi_and_normal_and_uv_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<RayIntersection<N, V>> {
        if self.uvs().is_none() || na::dim::<P>() != 3 {
            return self.toi_and_normal_with_ray(ray, solid);
        }

        let mut cost_fn = MeshRayToiAndNormalAndUVsCostFn { mesh: self, ray: ray };
        let cast = self.bvt().best_first_search(&mut cost_fn);

        match cast {
            None                => None,
            Some((best, inter)) => {
                let toi = inter.ref0().toi;
                let n   = inter.ref0().normal.clone();
                let uv  = inter.val1(); // barycentric coordinates to compute the exact uvs.

                let ibest = *best * 3;
                let is    = self.indices().slice(ibest, ibest + 3);
                let uvs   = self.uvs().as_ref().unwrap();

                let uv1 = uvs.deref()[is[0]];
                let uv2 = uvs.deref()[is[1]];
                let uv3 = uvs.deref()[is[2]];

                let uvx = uv1.x * uv.x + uv2.x * uv.y + uv3.x * uv.z;
                let uvy = uv1.y * uv.x + uv2.y * uv.y + uv3.y * uv.z;

                // XXX: this interpolation should be done on the two other ray cast too!
                match *self.normals() {
                    None         => {
                        Some(RayIntersection::new_with_uvs(toi, n, Some(Pnt2::new(uvx, uvy))))
                    },
                    Some(ref ns) => {
                        let n1 = &ns.deref()[is[0]];
                        let n2 = &ns.deref()[is[1]];
                        let n3 = &ns.deref()[is[2]];

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

impl<N, P, V, E, M> RayCast<N, P, V, M> for Mesh<N, P, V, E>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> + Rotate<V>,
          E: MeshElement<P> + LocalRayCast<N, P, V> {
}


/*
 * Costs functions.
 */
struct MeshRayToiCostFn<'a, N: 'a, P: 'a, V: 'a, E: 'a> {
    mesh:  &'a Mesh<N, P, V, E>,
    ray:   &'a Ray<P, V>
}

impl<'a, N, P, V, E> BVTCostFn<N, uint, AABB<P>, N> for MeshRayToiCostFn<'a, N, P, V, E>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          E: MeshElement<P> + LocalRayCast<N, P, V> {
    #[inline]
    fn compute_bv_cost(&mut self, aabb: &AABB<P>) -> Option<N> {
        aabb.toi_with_ray(self.ray, true)
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &uint) -> Option<(N, N)> {
        self.mesh.element_at(*b).toi_with_ray(self.ray, true).map(|toi| (toi, toi))
    }
}

struct MeshRayToiAndNormalCostFn<'a, N: 'a, P: 'a, V: 'a, E: 'a> {
    mesh:  &'a Mesh<N, P, V, E>,
    ray:   &'a Ray<P, V>
}

impl<'a, N, P, V, E> BVTCostFn<N, uint, AABB<P>, RayIntersection<N, V>>
for MeshRayToiAndNormalCostFn<'a, N, P, V, E>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          E: MeshElement<P> + LocalRayCast<N, P, V> {
    #[inline]
    fn compute_bv_cost(&mut self, aabb: &AABB<P>) -> Option<N> {
        aabb.toi_with_ray(self.ray, true)
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &uint) -> Option<(N, RayIntersection<N, V>)> {
        self.mesh.element_at(*b).toi_and_normal_with_ray(self.ray, true).map(|inter| (inter.toi, inter))
    }
}

struct MeshRayToiAndNormalAndUVsCostFn<'a, N: 'a, P: 'a, V: 'a, E: 'a> {
    mesh:  &'a Mesh<N, P, V, E>,
    ray:   &'a Ray<P, V>
}

impl<'a, N, P, V, E> BVTCostFn<N, uint, AABB<P>, (RayIntersection<N, V>, Vec3<N>)>
for MeshRayToiAndNormalAndUVsCostFn<'a, N, P, V, E>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          E: MeshElement<P> + LocalRayCast<N, P, V> {
    #[inline]
    fn compute_bv_cost(&mut self, aabb: &AABB<P>) -> Option<N> {
        aabb.toi_with_ray(self.ray, true)
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &uint) -> Option<(N, (RayIntersection<N, V>, Vec3<N>))> {
        let vs = self.mesh.vertices().as_slice();
        let i  = *b * 3;
        let is = self.mesh.indices().slice(i, i + 3);

        let a = &vs[is[0]];
        let b = &vs[is[1]];
        let c = &vs[is[2]];

        ray::triangle_ray_intersection(a, b, c, self.ray).map(|inter| (inter.ref0().toi.clone(), inter))
    }
}
