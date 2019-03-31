use crate::bounding_volume::AABB;
use crate::math::Isometry;
use na::{Point2, RealField, Vector3};
use crate::partitioning::{BestFirstBVVisitStatus, BestFirstDataVisitStatus, BestFirstVisitor};
use crate::query::{ray_internal, Ray, RayCast, RayIntersection};
use crate::shape::{CompositeShape, TriMesh, FeatureId};

impl<N: RealField> RayCast<N> for TriMesh<N> {
    #[inline]
    fn toi_with_ray(&self, m: &Isometry<N>, ray: &Ray<N>, _: bool) -> Option<N> {
        let ls_ray = ray.inverse_transform_by(m);

        let mut visitor = TriMeshRayToiVisitor {
            mesh: self,
            ray: &ls_ray,
        };

        self.bvh().best_first_search(&mut visitor)
    }

    #[inline]
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        _: bool,
    ) -> Option<RayIntersection<N>>
    {
        let ls_ray = ray.inverse_transform_by(m);

        let mut visitor = TriMeshRayToiAndNormalVisitor {
            mesh: self,
            ray: &ls_ray,
        };

        self.bvh().best_first_search(&mut visitor).map(|(best, mut res)| {
            if let FeatureId::Face(1) = res.feature {
                res.feature = FeatureId::Face(best + self.faces().len());
            } else {
                res.feature = FeatureId::Face(best);
            }

            res.normal = m * res.normal;
            res
        })
    }

    fn toi_and_normal_and_uv_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        solid: bool,
    ) -> Option<RayIntersection<N>>
    {
        if self.uvs().is_none() {
            return self.toi_and_normal_with_ray(m, ray, solid);
        }

        let ls_ray = ray.inverse_transform_by(m);

        let mut visitor = TriMeshRayToiAndNormalAndUVsVisitor {
            mesh: self,
            ray: &ls_ray,
        };
        let cast = self.bvh().best_first_search(&mut visitor);

        match cast {
            None => None,
            Some((best, inter, uv)) => {
                let toi = inter.toi;
                let n = inter.normal;

                let idx = &self.faces()[best].indices;
                let uvs = self.uvs().unwrap();

                let uv1 = uvs[idx[0]];
                let uv2 = uvs[idx[1]];
                let uv3 = uvs[idx[2]];

                let uvx = uv1.x * uv.x + uv2.x * uv.y + uv3.x * uv.z;
                let uvy = uv1.y * uv.x + uv2.y * uv.y + uv3.y * uv.z;

                let feature = if let FeatureId::Face(1) = inter.feature {
                    FeatureId::Face(best + self.faces().len())
                } else {
                    FeatureId::Face(best)
                };

                Some(RayIntersection::new_with_uvs(
                    toi,
                    m * n,
                    feature,
                    Some(Point2::new(uvx, uvy)),
                ))
            }
        }
    }
}

/*
 * Costs functions.
 */
struct TriMeshRayToiVisitor<'a, N: 'a + RealField> {
    mesh: &'a TriMesh<N>,
    ray: &'a Ray<N>,
}

impl<'a, N: RealField> BestFirstVisitor<N, usize, AABB<N>> for TriMeshRayToiVisitor<'a, N> {
    type Result = N;

    #[inline]
    fn visit_bv(&mut self, aabb: &AABB<N>) -> BestFirstBVVisitStatus<N> {
        match aabb.toi_with_ray(&Isometry::identity(), self.ray, true) {
            Some(toi) => BestFirstBVVisitStatus::ContinueWithCost(toi),
            None => BestFirstBVVisitStatus::Stop,
        }
    }

    #[inline]
    fn visit_data(&mut self, b: &usize) -> BestFirstDataVisitStatus<N, N> {
        match self
            .mesh
            .triangle_at(*b)
            .toi_with_ray(&Isometry::identity(), self.ray, true)
        {
            Some(toi) => BestFirstDataVisitStatus::ContinueWithResult(toi, toi),
            None => BestFirstDataVisitStatus::Continue,
        }
    }
}

struct TriMeshRayToiAndNormalVisitor<'a, N: 'a + RealField> {
    mesh: &'a TriMesh<N>,
    ray: &'a Ray<N>,
}

impl<'a, N: RealField> BestFirstVisitor<N, usize, AABB<N>> for TriMeshRayToiAndNormalVisitor<'a, N> {
    type Result = (usize, RayIntersection<N>);

    #[inline]
    fn visit_bv(&mut self, aabb: &AABB<N>) -> BestFirstBVVisitStatus<N> {
        match aabb.toi_with_ray(&Isometry::identity(), self.ray, true) {
            Some(toi) => BestFirstBVVisitStatus::ContinueWithCost(toi),
            None => BestFirstBVVisitStatus::Stop,
        }
    }

    #[inline]
    fn visit_data(&mut self, b: &usize) -> BestFirstDataVisitStatus<N, (usize, RayIntersection<N>)> {
        match self.mesh.triangle_at(*b).toi_and_normal_with_ray(
            &Isometry::identity(),
            self.ray,
            true,
        ) {
            Some(inter) => BestFirstDataVisitStatus::ContinueWithResult(inter.toi, (*b, inter)),
            None => BestFirstDataVisitStatus::Continue,
        }
    }
}

struct TriMeshRayToiAndNormalAndUVsVisitor<'a, N: 'a + RealField> {
    mesh: &'a TriMesh<N>,
    ray: &'a Ray<N>,
}

impl<'a, N: RealField> BestFirstVisitor<N, usize, AABB<N>>
    for TriMeshRayToiAndNormalAndUVsVisitor<'a, N>
{
    type Result = (usize, RayIntersection<N>, Vector3<N>);

    #[inline]
    fn visit_bv(&mut self, aabb: &AABB<N>) -> BestFirstBVVisitStatus<N> {
        match aabb.toi_with_ray(&Isometry::identity(), self.ray, true) {
            Some(toi) => BestFirstBVVisitStatus::ContinueWithCost(toi),
            None => BestFirstBVVisitStatus::Stop,
        }
    }

    #[inline]
    fn visit_data(
        &mut self,
        i: &usize,
    ) -> BestFirstDataVisitStatus<N, (usize, RayIntersection<N>, Vector3<N>)>
    {
        let vs = self.mesh.points();
        let idx = self.mesh.faces()[*i].indices;

        let a = &vs[idx[0]];
        let b = &vs[idx[1]];
        let c = &vs[idx[2]];

        match ray_internal::triangle_ray_intersection(a, b, c, self.ray) {
            Some(inter) => {
                BestFirstDataVisitStatus::ContinueWithResult(inter.0.toi, (*i, inter.0, inter.1))
            }
            None => BestFirstDataVisitStatus::Continue,
        }
    }
}
