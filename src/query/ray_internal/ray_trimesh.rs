use std::ops::Index;
use num::Zero;

use alga::linear::NormedSpace;
use na::{self, Real, Point2, Vector3};

use query::{ray_internal, Ray, RayCast, RayIntersection};
use shape::{TriMesh, CompositeShape};
use bounding_volume::AABB;
use partitioning::BVTCostFn;
use math::{Isometry, Point};

impl<N: Real> RayCast<N> for TriMesh<N> {
    #[inline]
    fn toi_with_ray(&self, m: &Isometry<N>, ray: &Ray<N>, _: bool) -> Option<N> {
        let ls_ray = ray.inverse_transform_by(m);

        let mut cost_fn = TriMeshRayToiCostFn {
            mesh: self,
            ray: &ls_ray,
        };

        self.bvt()
            .best_first_search(&mut cost_fn)
            .map(|(_, res)| res)
    }

    #[inline]
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        _: bool,
    ) -> Option<RayIntersection<N>> {
        let ls_ray = ray.inverse_transform_by(m);

        let mut cost_fn = TriMeshRayToiAndNormalCostFn {
            mesh: self,
            ray: &ls_ray,
        };

        self.bvt()
            .best_first_search(&mut cost_fn)
            .map(|(_, mut res)| {
                res.normal = m * res.normal;
                res
            })
    }

    fn toi_and_normal_and_uv_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        solid: bool,
    ) -> Option<RayIntersection<N>> {
        if self.uvs().is_none() {
            return self.toi_and_normal_with_ray(m, ray, solid);
        }

        let ls_ray = ray.inverse_transform_by(m);

        let mut cost_fn = TriMeshRayToiAndNormalAndUVsCostFn {
            mesh: self,
            ray: &ls_ray,
        };
        let cast = self.bvt().best_first_search(&mut cost_fn);

        match cast {
            None => None,
            Some((best, inter)) => {
                let toi = inter.0.toi;
                let n = inter.0.normal;
                let uv = inter.1; // barycentric coordinates to compute the exact uvs.

                let idx = &self.indices()[*best];
                let uvs = self.uvs().as_ref().unwrap();

                let uv1 = uvs[idx[0]];
                let uv2 = uvs[idx[1]];
                let uv3 = uvs[idx[2]];

                let uvx = uv1.x * uv.x + uv2.x * uv.y + uv3.x * uv.z;
                let uvy = uv1.y * uv.x + uv2.y * uv.y + uv3.y * uv.z;

                Some(RayIntersection::new_with_uvs(toi, m * n, Some(Point2::new(uvx, uvy))))
            }
        }
    }
}

/*
 * Costs functions.
 */
struct TriMeshRayToiCostFn<'a, N: 'a + Real> {
    mesh: &'a TriMesh<N>,
    ray: &'a Ray<N>,
}

impl<'a, N: Real> BVTCostFn<N, usize, AABB<N>> for TriMeshRayToiCostFn<'a, N> {
    type UserData = N;

    #[inline]
    fn compute_bv_cost(&mut self, aabb: &AABB<N>) -> Option<N> {
        aabb.toi_with_ray(&Isometry::identity(), self.ray, true)
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &usize) -> Option<(N, N)> {
        self.mesh
            .triangle_at(*b)
            .toi_with_ray(&Isometry::identity(), self.ray, true)
            .map(|toi| (toi, toi))
    }
}

struct TriMeshRayToiAndNormalCostFn<'a, N: 'a + Real> {
    mesh: &'a TriMesh<N>,
    ray: &'a Ray<N>,
}

impl<'a, N: Real> BVTCostFn<N, usize, AABB<N>> for TriMeshRayToiAndNormalCostFn<'a, N> {
    type UserData = RayIntersection<N>;

    #[inline]
    fn compute_bv_cost(&mut self, aabb: &AABB<N>) -> Option<N> {
        aabb.toi_with_ray(&Isometry::identity(), self.ray, true)
    }

    #[inline]
    fn compute_b_cost(&mut self, b: &usize) -> Option<(N, RayIntersection<N>)> {
        self.mesh
            .triangle_at(*b)
            .toi_and_normal_with_ray(&Isometry::identity(), self.ray, true)
            .map(|inter| (inter.toi, inter))
    }
}

struct TriMeshRayToiAndNormalAndUVsCostFn<'a, N: 'a + Real> {
    mesh: &'a TriMesh<N>,
    ray: &'a Ray<N>,
}

impl<'a, N: Real> BVTCostFn<N, usize, AABB<N>> for TriMeshRayToiAndNormalAndUVsCostFn<'a, N>{
    type UserData = (RayIntersection<N>, Vector3<N>);

    #[inline]
    fn compute_bv_cost(&mut self, aabb: &AABB<N>) -> Option<N> {
        aabb.toi_with_ray(&Isometry::identity(), self.ray, true)
    }

    #[inline]
    fn compute_b_cost(
        &mut self,
        b: &usize,
    ) -> Option<(N, (RayIntersection<N>, Vector3<N>))> {
        let vs = &self.mesh.vertices()[..];
        let idx = &self.mesh.indices()[*b];

        let a = &vs[idx[0]];
        let b = &vs[idx[1]];
        let c = &vs[idx[2]];

        ray_internal::triangle_ray_intersection(a, b, c, self.ray).map(|inter| (inter.0.toi, inter))
    }
}
