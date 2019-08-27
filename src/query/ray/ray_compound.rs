use crate::bounding_volume::AABB;
use crate::math::Isometry;
use crate::partitioning::{BestFirstVisitStatus, BestFirstVisitor, BVH};
use crate::query::{Ray, RayCast, RayIntersection};
use crate::shape::Compound;
use na::RealField;

// XXX: if solid == false, this might return internal intersection.
impl<N: RealField> RayCast<N> for Compound<N> {
    fn toi_with_ray(&self, m: &Isometry<N>, ray: &Ray<N>, solid: bool) -> Option<N> {
        let ls_ray = ray.inverse_transform_by(m);

        let mut visitor = CompoundRayToiVisitor {
            compound: self,
            ray: &ls_ray,
            solid: solid,
        };

        self.bvt().best_first_search(&mut visitor).map(|res| res.1)
    }

    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        solid: bool,
    ) -> Option<RayIntersection<N>> {
        let ls_ray = ray.inverse_transform_by(m);

        let mut visitor = CompoundRayToiAndNormalVisitor {
            compound: self,
            ray: &ls_ray,
            solid: solid,
        };

        self.bvt()
            .best_first_search(&mut visitor)
            .map(|(_, mut res)| {
                res.normal = m * res.normal;
                res
            })
    }

    // XXX: We have to implement toi_and_normal_and_uv_with_ray! Otherwise, no uv will be computed
    // for any of the sub-shapes.
}

/*
 * Costs functions.
 */
struct CompoundRayToiVisitor<'a, N: 'a + RealField> {
    compound: &'a Compound<N>,
    ray: &'a Ray<N>,
    solid: bool,
}

impl<'a, N: RealField> BestFirstVisitor<N, usize, AABB<N>> for CompoundRayToiVisitor<'a, N> {
    type Result = N;

    #[inline]
    fn visit(
        &mut self,
        best: N,
        aabb: &AABB<N>,
        data: Option<&usize>,
    ) -> BestFirstVisitStatus<N, Self::Result> {
        if let Some(toi) = aabb.toi_with_ray(&Isometry::identity(), self.ray, true) {
            let mut res = BestFirstVisitStatus::Continue {
                cost: toi,
                result: None,
            };

            if let Some(b) = data {
                if toi < best {
                    let elt = &self.compound.shapes()[*b];
                    if let Some(toi) = elt.1.toi_with_ray(&elt.0, self.ray, self.solid) {
                        res = BestFirstVisitStatus::Continue {
                            cost: toi,
                            result: Some(toi),
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

struct CompoundRayToiAndNormalVisitor<'a, N: 'a + RealField> {
    compound: &'a Compound<N>,
    ray: &'a Ray<N>,
    solid: bool,
}

impl<'a, N: RealField> BestFirstVisitor<N, usize, AABB<N>>
    for CompoundRayToiAndNormalVisitor<'a, N>
{
    type Result = RayIntersection<N>;

    #[inline]
    fn visit(
        &mut self,
        best: N,
        aabb: &AABB<N>,
        data: Option<&usize>,
    ) -> BestFirstVisitStatus<N, Self::Result> {
        if let Some(toi) = aabb.toi_with_ray(&Isometry::identity(), self.ray, true) {
            let mut res = BestFirstVisitStatus::Continue {
                cost: toi,
                result: None,
            };

            if let Some(b) = data {
                if toi < best {
                    let elt = &self.compound.shapes()[*b];
                    if let Some(toi) = elt.1.toi_and_normal_with_ray(&elt.0, self.ray, self.solid) {
                        res = BestFirstVisitStatus::Continue {
                            cost: toi.toi,
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
