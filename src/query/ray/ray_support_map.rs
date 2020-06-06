use na::{self, RealField};

use crate::math::Isometry;
#[cfg(feature = "dim2")]
use crate::query;
use crate::query::algorithms::{gjk, CSOPoint, VoronoiSimplex};
use crate::query::{Ray, RayCast, RayIntersection};
#[cfg(feature = "dim2")]
use crate::shape::ConvexPolygon;
use crate::shape::{Capsule, FeatureId, Segment, SupportMap};
#[cfg(feature = "dim3")]
use crate::shape::{Cone, ConvexHull, Cylinder};

/// Cast a ray on a shape using the GJK algorithm.
pub fn ray_intersection_with_support_map_with_params<N, G: ?Sized>(
    m: &Isometry<N>,
    shape: &G,
    simplex: &mut VoronoiSimplex<N>,
    ray: &Ray<N>,
    max_toi: N,
    solid: bool,
) -> Option<RayIntersection<N>>
where
    N: RealField,
    G: SupportMap<N>,
{
    let supp = shape.support_point(m, &-ray.dir);
    simplex.reset(CSOPoint::single_point(supp - ray.origin.coords));

    let inter = gjk::cast_ray(m, shape, simplex, ray, max_toi);

    if !solid {
        inter.and_then(|(toi, normal)| {
            if toi.is_zero() {
                // the ray is inside of the shape.
                let ndir = ray.dir.normalize();
                let supp = shape.support_point(m, &ndir);
                let eps = na::convert(0.001f64);
                let shift = (supp - ray.origin).dot(&ndir) + eps;
                let new_ray = Ray::new(ray.origin + ndir * shift, -ray.dir);

                // FIXME: replace by? : simplex.translate_by(&(ray.origin - new_ray.origin));
                simplex.reset(CSOPoint::single_point(supp - new_ray.origin.coords));

                gjk::cast_ray(m, shape, simplex, &new_ray, shift + eps).and_then(|(toi, normal)| {
                    let toi = shift - toi;
                    if toi <= max_toi {
                        Some(RayIntersection::new(toi, normal, FeatureId::Unknown))
                    } else {
                        None
                    }
                })
            } else {
                Some(RayIntersection::new(toi, normal, FeatureId::Unknown))
            }
        })
    } else {
        inter.map(|(toi, normal)| RayIntersection::new(toi, normal, FeatureId::Unknown))
    }
}

#[cfg(feature = "dim3")]
impl<N: RealField> RayCast<N> for Cylinder<N> {
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        max_toi: N,
        solid: bool,
    ) -> Option<RayIntersection<N>> {
        let ls_ray = ray.inverse_transform_by(m);

        ray_intersection_with_support_map_with_params(
            &Isometry::identity(),
            self,
            &mut VoronoiSimplex::new(),
            &ls_ray,
            max_toi,
            solid,
        )
        .map(|mut res| {
            res.normal = m * res.normal;
            res
        })
    }
}

#[cfg(feature = "dim3")]
impl<N: RealField> RayCast<N> for Cone<N> {
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        max_toi: N,
        solid: bool,
    ) -> Option<RayIntersection<N>> {
        let ls_ray = ray.inverse_transform_by(m);

        ray_intersection_with_support_map_with_params(
            &Isometry::identity(),
            self,
            &mut VoronoiSimplex::new(),
            &ls_ray,
            max_toi,
            solid,
        )
        .map(|mut res| {
            res.normal = m * res.normal;
            res
        })
    }
}

impl<N: RealField> RayCast<N> for Capsule<N> {
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        max_toi: N,
        solid: bool,
    ) -> Option<RayIntersection<N>> {
        let ls_ray = ray.inverse_transform_by(m);

        ray_intersection_with_support_map_with_params(
            &Isometry::identity(),
            self,
            &mut VoronoiSimplex::new(),
            &ls_ray,
            max_toi,
            solid,
        )
        .map(|mut res| {
            res.normal = m * res.normal;
            res
        })
    }
}

#[cfg(feature = "dim3")]
impl<N: RealField> RayCast<N> for ConvexHull<N> {
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        max_toi: N,
        solid: bool,
    ) -> Option<RayIntersection<N>> {
        let ls_ray = ray.inverse_transform_by(m);

        ray_intersection_with_support_map_with_params(
            &Isometry::identity(),
            self,
            &mut VoronoiSimplex::new(),
            &ls_ray,
            max_toi,
            solid,
        )
        .map(|mut res| {
            res.normal = m * res.normal;
            res
        })
    }
}

#[cfg(feature = "dim2")]
impl<N: RealField> RayCast<N> for ConvexPolygon<N> {
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        max_toi: N,
        solid: bool,
    ) -> Option<RayIntersection<N>> {
        let ls_ray = ray.inverse_transform_by(m);

        ray_intersection_with_support_map_with_params(
            &Isometry::identity(),
            self,
            &mut VoronoiSimplex::new(),
            &ls_ray,
            max_toi,
            solid,
        )
        .map(|mut res| {
            res.normal = m * res.normal;
            res
        })
    }
}

#[allow(unused_variables)]
impl<N: RealField> RayCast<N> for Segment<N> {
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        max_toi: N,
        solid: bool,
    ) -> Option<RayIntersection<N>> {
        #[cfg(feature = "dim2")]
        {
            let seg = self.transformed(m);
            let seg_dir = seg.scaled_direction();
            let (s, t, parallel) = query::closest_points_line_line_parameters_eps(
                &ray.origin,
                &ray.dir,
                &seg.a,
                &seg_dir,
                N::default_epsilon(),
            );

            if parallel {
                // The lines are parallel, we have to distinguish
                // the case where there is no intersection at all
                // from the case where the line are collinear.
                let dpos = seg.a - ray.origin;
                let normal = seg.scaled_normal();

                if dpos.dot(&normal).abs() < N::default_epsilon() {
                    // The rays and the segment are collinear.
                    let dist1 = dpos.dot(&ray.dir);
                    let dist2 = dist1 + seg_dir.dot(&ray.dir);

                    match (dist1 >= N::zero(), dist2 >= N::zero()) {
                        (true, true) => {
                            if dist1 <= dist2 {
                                Some(RayIntersection::new(
                                    dist1 / ray.dir.norm_squared(),
                                    normal,
                                    FeatureId::Vertex(0),
                                ))
                            } else {
                                Some(RayIntersection::new(
                                    dist2 / ray.dir.norm_squared(),
                                    normal,
                                    FeatureId::Vertex(1),
                                ))
                            }
                        }
                        (true, false) | (false, true) => {
                            // The ray origin lies on the segment.
                            Some(RayIntersection::new(N::zero(), normal, FeatureId::Face(0)))
                        }
                        (false, false) => {
                            // The segment is behind the ray.
                            None
                        }
                    }
                } else {
                    // The rays never intersect.
                    None
                }
            } else if s >= N::zero() && t >= N::zero() && t <= N::one() {
                let normal = seg.scaled_normal();

                if normal.dot(&ray.dir) > N::zero() {
                    Some(RayIntersection::new(s, -normal, FeatureId::Face(1)))
                } else {
                    Some(RayIntersection::new(s, normal, FeatureId::Face(0)))
                }
            } else {
                // The closest points are outside of
                // the ray or segment bounds.
                None
            }
        }
        #[cfg(feature = "dim3")]
        {
            let ls_ray = ray.inverse_transform_by(m);

            // XXX: implement an analytic solution for 3D too.
            ray_intersection_with_support_map_with_params(
                &Isometry::identity(),
                self,
                &mut VoronoiSimplex::new(),
                &ls_ray,
                max_toi,
                solid,
            )
            .map(|mut res| {
                res.normal = m * res.normal;
                res
            })
        }
    }
}
