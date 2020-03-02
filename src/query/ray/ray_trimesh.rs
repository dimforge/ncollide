use crate::bounding_volume::AABB;
use crate::math::Isometry;
use crate::partitioning::{BestFirstVisitStatus, BestFirstVisitor};
use crate::query::{self, Ray, RayCast, RayIntersection};
use crate::shape::{CompositeShape, FeatureId, TriMesh};
use na::{Point2, RealField, Vector3};

impl<N: RealField> RayCast<N> for TriMesh<N> {
    #[inline]
    fn toi_with_ray(&self, m: &Isometry<N>, ray: &Ray<N>, max_toi: N, _: bool) -> Option<N> {
        let ls_ray = ray.inverse_transform_by(m);

        let mut visitor = TriMeshRayToiVisitor {
            mesh: self,
            ray: &ls_ray,
            max_toi,
        };

        self.bvh().best_first_search(&mut visitor).map(|res| res.1)
    }

    #[inline]
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        max_toi: N,
        _: bool,
    ) -> Option<RayIntersection<N>> {
        let ls_ray = ray.inverse_transform_by(m);

        let mut visitor = TriMeshRayToiAndNormalVisitor {
            mesh: self,
            ray: &ls_ray,
            max_toi,
        };

        self.bvh()
            .best_first_search(&mut visitor)
            .map(|(_, (best, mut res))| {
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
        max_toi: N,
        solid: bool,
    ) -> Option<RayIntersection<N>> {
        if self.uvs().is_none() {
            return self.toi_and_normal_with_ray(m, ray, max_toi, solid);
        }

        let ls_ray = ray.inverse_transform_by(m);

        let mut visitor = TriMeshRayToiAndNormalAndUVsVisitor {
            mesh: self,
            ray: &ls_ray,
            max_toi,
        };
        let cast = self.bvh().best_first_search(&mut visitor);

        cast.map(|(_, (best, inter, uv))| {
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

            RayIntersection::new_with_uvs(toi, m * n, feature, Some(Point2::new(uvx, uvy)))
        })
    }
}

/*
 * Costs functions.
 */
struct TriMeshRayToiVisitor<'a, N: 'a + RealField> {
    mesh: &'a TriMesh<N>,
    ray: &'a Ray<N>,
    max_toi: N,
}

impl<'a, N: RealField> BestFirstVisitor<N, usize, AABB<N>> for TriMeshRayToiVisitor<'a, N> {
    type Result = N;

    #[inline]
    fn visit(
        &mut self,
        best: N,
        aabb: &AABB<N>,
        data: Option<&usize>,
    ) -> BestFirstVisitStatus<N, Self::Result> {
        if let Some(toi) = aabb.toi_with_ray(&Isometry::identity(), self.ray, self.max_toi, true) {
            let mut res = BestFirstVisitStatus::Continue {
                cost: toi,
                result: None,
            };

            if let Some(b) = data {
                if toi < best {
                    // FIXME: optimize this by not using Isometry identity.
                    let triangle = self.mesh.triangle_at(*b);
                    if let Some(toi) = triangle.toi_with_ray(&Isometry::identity(), self.ray, self.max_toi, true)
                    {
                        res = BestFirstVisitStatus::Continue {
                            cost: toi,
                            result: Some(toi),
                        }
                    }
                }
            }

            res
        } else {
            BestFirstVisitStatus::Stop
        }
    }
}

struct TriMeshRayToiAndNormalVisitor<'a, N: 'a + RealField> {
    mesh: &'a TriMesh<N>,
    ray: &'a Ray<N>,
    max_toi: N,
}

impl<'a, N: RealField> BestFirstVisitor<N, usize, AABB<N>>
    for TriMeshRayToiAndNormalVisitor<'a, N>
{
    type Result = (usize, RayIntersection<N>);

    #[inline]
    fn visit(
        &mut self,
        best: N,
        aabb: &AABB<N>,
        data: Option<&usize>,
    ) -> BestFirstVisitStatus<N, Self::Result> {
        if let Some(toi) = aabb.toi_with_ray(&Isometry::identity(), self.ray, self.max_toi, true) {
            let mut res = BestFirstVisitStatus::Continue {
                cost: toi,
                result: None,
            };

            if let Some(b) = data {
                if toi < best {
                    // FIXME: optimize this by not using Isometry identity.
                    let triangle = self.mesh.triangle_at(*b);
                    if let Some(toi) =
                        triangle.toi_and_normal_with_ray(&Isometry::identity(), self.ray, self.max_toi, true)
                    {
                        res = BestFirstVisitStatus::Continue {
                            cost: toi.toi,
                            result: Some((*b, toi)),
                        };
                    }
                }
            }

            res
        } else {
            BestFirstVisitStatus::Stop
        }
    }
}

struct TriMeshRayToiAndNormalAndUVsVisitor<'a, N: 'a + RealField> {
    mesh: &'a TriMesh<N>,
    ray: &'a Ray<N>,
    max_toi: N,
}

impl<'a, N: RealField> BestFirstVisitor<N, usize, AABB<N>>
    for TriMeshRayToiAndNormalAndUVsVisitor<'a, N>
{
    type Result = (usize, RayIntersection<N>, Vector3<N>);

    #[inline]
    fn visit(
        &mut self,
        best: N,
        aabb: &AABB<N>,
        data: Option<&usize>,
    ) -> BestFirstVisitStatus<N, Self::Result> {
        if let Some(toi) = aabb.toi_with_ray(&Isometry::identity(), self.ray, self.max_toi, true) {
            let mut res = BestFirstVisitStatus::Continue {
                cost: toi,
                result: None,
            };

            if let Some(i) = data {
                if toi < best {
                    let vs = self.mesh.points();
                    let idx = self.mesh.faces()[*i].indices;

                    let a = &vs[idx[0]];
                    let b = &vs[idx[1]];
                    let c = &vs[idx[2]];

                    if let Some(inter) = query::ray_intersection_with_triangle(a, b, c, self.ray) {
                        if inter.0.toi <= self.max_toi {
                            res = BestFirstVisitStatus::Continue {
                                cost: inter.0.toi,
                                result: Some((*i, inter.0, inter.1)),
                            };
                        }
                    }
                }
            }

            res
        } else {
            BestFirstVisitStatus::Stop
        }
    }
}
