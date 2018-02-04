use num::Zero;

use alga::general::Id;
use na;

use query::algorithms::gjk;
use query::algorithms::{Simplex, JohnsonSimplex, VoronoiSimplex2, VoronoiSimplex3};
use query::{Ray, RayCast, RayIntersection};
use shape::{Capsule, Cone, ConvexHull, Cylinder, MinkowskiSum, Segment, SupportMap};
use math::{Isometry, Point};

/// Cast a ray on a shape using the GJK algorithm.
pub fn implicit_toi_and_normal_with_ray<P, M, S, G: ?Sized>(
    m: &M,
    shape: &G,
    simplex: &mut S,
    ray: &Ray<P>,
    solid: bool,
) -> Option<RayIntersection<P::Vector>>
where
    P: Point,
    M: Isometry<P>,
    S: Simplex<P>,
    G: SupportMap<P, M>,
{
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
                    simplex.reset(supp + (-new_ray.origin.coordinates()));

                    gjk::cast_ray(m, shape, simplex, &new_ray)
                        .map(|(toi, normal)| RayIntersection::new(shift - toi, normal))
                } else {
                    Some(RayIntersection::new(toi, normal))
                }
            }
        }
    } else {
        inter.map(|(toi, normal)| RayIntersection::new(toi, normal))
    }
}

impl<P, M> RayCast<P, M> for Cylinder<P::Real>
where
    P: Point,
    M: Isometry<P>,
{
    fn toi_and_normal_with_ray(
        &self,
        m: &M,
        ray: &Ray<P>,
        solid: bool,
    ) -> Option<RayIntersection<P::Vector>> {
        let ls_ray = ray.inverse_transform_by(m);

        if na::dimension::<P::Vector>() == 2 {
            implicit_toi_and_normal_with_ray(
                &Id::new(),
                self,
                &mut VoronoiSimplex2::<P>::new(),
                &ls_ray,
                solid,
            ).map(|mut res| {
                res.normal = m.rotate_vector(&res.normal);
                res
            })
        } else if na::dimension::<P::Vector>() == 3 {
            implicit_toi_and_normal_with_ray(
                &Id::new(),
                self,
                &mut VoronoiSimplex3::<P>::new(),
                &ls_ray,
                solid,
            ).map(|mut res| {
                res.normal = m.rotate_vector(&res.normal);
                res
            })
        } else {
            implicit_toi_and_normal_with_ray(
                &Id::new(),
                self,
                &mut JohnsonSimplex::<P>::new_w_tls(),
                &ls_ray,
                solid,
            ).map(|mut res| {
                res.normal = m.rotate_vector(&res.normal);
                res
            })
        }
    }
}

impl<P, M> RayCast<P, M> for Cone<P::Real>
where
    P: Point,
    M: Isometry<P>,
{
    fn toi_and_normal_with_ray(
        &self,
        m: &M,
        ray: &Ray<P>,
        solid: bool,
    ) -> Option<RayIntersection<P::Vector>> {
        let ls_ray = ray.inverse_transform_by(m);

        if na::dimension::<P::Vector>() == 2 {
            implicit_toi_and_normal_with_ray(
                &Id::new(),
                self,
                &mut VoronoiSimplex2::<P>::new(),
                &ls_ray,
                solid,
            ).map(|mut res| {
                res.normal = m.rotate_vector(&res.normal);
                res
            })
        } else if na::dimension::<P::Vector>() == 3 {
            implicit_toi_and_normal_with_ray(
                &Id::new(),
                self,
                &mut VoronoiSimplex3::<P>::new(),
                &ls_ray,
                solid,
            ).map(|mut res| {
                res.normal = m.rotate_vector(&res.normal);
                res
            })
        } else {
            implicit_toi_and_normal_with_ray(
                &Id::new(),
                self,
                &mut JohnsonSimplex::<P>::new_w_tls(),
                &ls_ray,
                solid,
            ).map(|mut res| {
                res.normal = m.rotate_vector(&res.normal);
                res
            })
        }
    }
}

impl<P, M> RayCast<P, M> for Capsule<P::Real>
where
    P: Point,
    M: Isometry<P>,
{
    fn toi_and_normal_with_ray(
        &self,
        m: &M,
        ray: &Ray<P>,
        solid: bool,
    ) -> Option<RayIntersection<P::Vector>> {
        let ls_ray = ray.inverse_transform_by(m);

        if na::dimension::<P::Vector>() == 2 {
            implicit_toi_and_normal_with_ray(
                &Id::new(),
                self,
                &mut VoronoiSimplex2::<P>::new(),
                &ls_ray,
                solid,
            ).map(|mut res| {
                res.normal = m.rotate_vector(&res.normal);
                res
            })
        } else if na::dimension::<P::Vector>() == 3 {
            implicit_toi_and_normal_with_ray(
                &Id::new(),
                self,
                &mut VoronoiSimplex3::<P>::new(),
                &ls_ray,
                solid,
            ).map(|mut res| {
                res.normal = m.rotate_vector(&res.normal);
                res
            })
        } else {
            implicit_toi_and_normal_with_ray(
                &Id::new(),
                self,
                &mut JohnsonSimplex::<P>::new_w_tls(),
                &ls_ray,
                solid,
            ).map(|mut res| {
                res.normal = m.rotate_vector(&res.normal);
                res
            })
        }
    }
}

impl<P, M> RayCast<P, M> for ConvexHull<P>
where
    P: Point,
    M: Isometry<P>,
{
    fn toi_and_normal_with_ray(
        &self,
        m: &M,
        ray: &Ray<P>,
        solid: bool,
    ) -> Option<RayIntersection<P::Vector>> {
        let ls_ray = ray.inverse_transform_by(m);

        if na::dimension::<P::Vector>() == 2 {
            implicit_toi_and_normal_with_ray(
                &Id::new(),
                self,
                &mut VoronoiSimplex2::<P>::new(),
                &ls_ray,
                solid,
            ).map(|mut res| {
                res.normal = m.rotate_vector(&res.normal);
                res
            })
        } else if na::dimension::<P::Vector>() == 3 {
            implicit_toi_and_normal_with_ray(
                &Id::new(),
                self,
                &mut VoronoiSimplex3::<P>::new(),
                &ls_ray,
                solid,
            ).map(|mut res| {
                res.normal = m.rotate_vector(&res.normal);
                res
            })
        } else {
            implicit_toi_and_normal_with_ray(
                &Id::new(),
                self,
                &mut JohnsonSimplex::<P>::new_w_tls(),
                &ls_ray,
                solid,
            ).map(|mut res| {
                res.normal = m.rotate_vector(&res.normal);
                res
            })
        }
    }
}

impl<P, M> RayCast<P, M> for Segment<P>
where
    P: Point,
    M: Isometry<P>,
{
    fn toi_and_normal_with_ray(
        &self,
        m: &M,
        ray: &Ray<P>,
        solid: bool,
    ) -> Option<RayIntersection<P::Vector>> {
        // XXX: optimize if na::dimension::<P>() == 2
        let ls_ray = ray.inverse_transform_by(m);

        if na::dimension::<P::Vector>() == 2 {
            implicit_toi_and_normal_with_ray(
                &Id::new(),
                self,
                &mut VoronoiSimplex2::<P>::new(),
                &ls_ray,
                solid,
            ).map(|mut res| {
                res.normal = m.rotate_vector(&res.normal);
                res
            })
        } else if na::dimension::<P::Vector>() == 3 {
            implicit_toi_and_normal_with_ray(
                &Id::new(),
                self,
                &mut VoronoiSimplex3::<P>::new(),
                &ls_ray,
                solid,
            ).map(|mut res| {
                res.normal = m.rotate_vector(&res.normal);
                res
            })
        } else {
            implicit_toi_and_normal_with_ray(
                &Id::new(),
                self,
                &mut JohnsonSimplex::<P>::new_w_tls(),
                &ls_ray,
                solid,
            ).map(|mut res| {
                res.normal = m.rotate_vector(&res.normal);
                res
            })
        }
    }
}

impl<'a, P, M, M2, G1: ?Sized, G2: ?Sized> RayCast<P, M2> for MinkowskiSum<'a, M, G1, G2>
where
    P: Point,
    M2: Isometry<P>,
    G1: SupportMap<P, M>,
    G2: SupportMap<P, M>,
{
    fn toi_and_normal_with_ray(
        &self,
        m: &M2,
        ray: &Ray<P>,
        solid: bool,
    ) -> Option<RayIntersection<P::Vector>> {
        let ls_ray = ray.inverse_transform_by(m);

        if na::dimension::<P::Vector>() == 2 {
            implicit_toi_and_normal_with_ray(
                &Id::new(),
                self,
                &mut VoronoiSimplex2::<P>::new(),
                &ls_ray,
                solid,
            ).map(|mut res| {
                res.normal = m.rotate_vector(&res.normal);
                res
            })
        } else if na::dimension::<P::Vector>() == 3 {
            implicit_toi_and_normal_with_ray(
                &Id::new(),
                self,
                &mut VoronoiSimplex3::<P>::new(),
                &ls_ray,
                solid,
            ).map(|mut res| {
                res.normal = m.rotate_vector(&res.normal);
                res
            })
        } else {
            implicit_toi_and_normal_with_ray(
                &Id::new(),
                self,
                &mut JohnsonSimplex::<P>::new_w_tls(),
                &ls_ray,
                solid,
            ).map(|mut res| {
                res.normal = m.rotate_vector(&res.normal);
                res
            })
        }
    }
}
