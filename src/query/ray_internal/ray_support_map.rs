use na::{self, Real};

use math::Isometry;
use query::algorithms::{gjk, CSOPoint, VoronoiSimplex};
use query::{Ray, RayCast, RayIntersection};
#[cfg(feature = "dim2")]
use shape::ConvexPolygon;
use shape::{Capsule, Segment, SupportMap, FeatureId};
#[cfg(feature = "dim3")]
use shape::{Cone, ConvexHull, Cylinder};

/// Cast a ray on a shape using the GJK algorithm.
pub fn implicit_toi_and_normal_with_ray<N, G: ?Sized>(
    m: &Isometry<N>,
    shape: &G,
    simplex: &mut VoronoiSimplex<N>,
    ray: &Ray<N>,
    solid: bool,
) -> Option<RayIntersection<N>>
where
    N: Real,
    G: SupportMap<N>,
{
    let supp = shape.support_point(m, &-ray.dir);
    simplex.reset(CSOPoint::single_point(supp - ray.origin.coords));

    let inter = gjk::cast_ray(m, shape, simplex, ray);

    if !solid {
        match inter {
            None => None,
            Some((toi, normal)) => {
                if toi.is_zero() {
                    // the ray is inside of the shape.
                    let ndir = na::normalize(&ray.dir);
                    let supp = shape.support_point(m, &ndir);
                    let shift = na::dot(&(supp - ray.origin), &ndir) + na::convert(0.001f64);
                    let new_ray = Ray::new(ray.origin + ndir * shift, -ray.dir);

                    // FIXME: replace by? : simplex.translate_by(&(ray.origin - new_ray.origin));
                    simplex.reset(CSOPoint::single_point(supp - new_ray.origin.coords));

                    gjk::cast_ray(m, shape, simplex, &new_ray)
                        .map(|(toi, normal)| RayIntersection::new(shift - toi, normal, FeatureId::Unknown))
                } else {
                    Some(RayIntersection::new(toi, normal, FeatureId::Unknown))
                }
            }
        }
    } else {
        inter.map(|(toi, normal)| RayIntersection::new(toi, normal, FeatureId::Unknown))
    }
}

#[cfg(feature = "dim3")]
impl<N: Real> RayCast<N> for Cylinder<N> {
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        solid: bool,
    ) -> Option<RayIntersection<N>>
    {
        let ls_ray = ray.inverse_transform_by(m);

        implicit_toi_and_normal_with_ray(
            &Isometry::identity(),
            self,
            &mut VoronoiSimplex::new(),
            &ls_ray,
            solid,
        )
        .map(|mut res| {
            res.normal = m * res.normal;
            res
        })
    }
}

#[cfg(feature = "dim3")]
impl<N: Real> RayCast<N> for Cone<N> {
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        solid: bool,
    ) -> Option<RayIntersection<N>>
    {
        let ls_ray = ray.inverse_transform_by(m);

        implicit_toi_and_normal_with_ray(
            &Isometry::identity(),
            self,
            &mut VoronoiSimplex::new(),
            &ls_ray,
            solid,
        )
        .map(|mut res| {
            res.normal = m * res.normal;
            res
        })
    }
}

impl<N: Real> RayCast<N> for Capsule<N> {
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        solid: bool,
    ) -> Option<RayIntersection<N>>
    {
        let ls_ray = ray.inverse_transform_by(m);

        implicit_toi_and_normal_with_ray(
            &Isometry::identity(),
            self,
            &mut VoronoiSimplex::new(),
            &ls_ray,
            solid,
        )
        .map(|mut res| {
            res.normal = m * res.normal;
            res
        })
    }
}

#[cfg(feature = "dim3")]
impl<N: Real> RayCast<N> for ConvexHull<N> {
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        solid: bool,
    ) -> Option<RayIntersection<N>>
    {
        let ls_ray = ray.inverse_transform_by(m);

        implicit_toi_and_normal_with_ray(
            &Isometry::identity(),
            self,
            &mut VoronoiSimplex::new(),
            &ls_ray,
            solid,
        )
        .map(|mut res| {
            res.normal = m * res.normal;
            res
        })
    }
}

#[cfg(feature = "dim2")]
impl<N: Real> RayCast<N> for ConvexPolygon<N> {
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        solid: bool,
    ) -> Option<RayIntersection<N>>
    {
        let ls_ray = ray.inverse_transform_by(m);

        implicit_toi_and_normal_with_ray(
            &Isometry::identity(),
            self,
            &mut VoronoiSimplex::new(),
            &ls_ray,
            solid,
        )
        .map(|mut res| {
            res.normal = m * res.normal;
            res
        })
    }
}

impl<N: Real> RayCast<N> for Segment<N> {
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        solid: bool,
    ) -> Option<RayIntersection<N>>
    {
        // XXX: implement an analytic solution
        let ls_ray = ray.inverse_transform_by(m);

        implicit_toi_and_normal_with_ray(
            &Isometry::identity(),
            self,
            &mut VoronoiSimplex::new(),
            &ls_ray,
            solid,
        )
        .map(|mut res| {
            res.normal = m * res.normal;
            res
        })
    }
}
