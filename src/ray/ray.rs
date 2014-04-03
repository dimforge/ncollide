//! Traits and structure needed to cast rays.

use nalgebra::na::{Rotate, Transform};
use math::{Scalar, Vect, Matrix};

/// A Ray.
#[deriving(Show, Encodable, Decodable)]
pub struct Ray {
    /// Starting point of the ray.
    pub orig: Vect,
    /// Direction of the ray.
    pub dir:  Vect
}

impl Ray {
    /// Creates a new ray starting from `orig` and with the direction `dir`. `dir` must be
    /// normalized.
    pub fn new(orig: Vect, dir: Vect) -> Ray {
        Ray {
            orig: orig,
            dir:  dir
        }
    }
}

/// Structure containing the result of a successful ray cast.
pub struct RayIntersection {
    /// The time of impact of the ray with the object.  The exact contact point can be computed
    /// with: `orig + dir * toi` where `orig` is the origin of the ray; `dir` is its direction and
    /// `toi` is the value of this field.
    pub toi:    Scalar,

    /// The normal at the intersection point.
    ///
    /// If the `toi` is exactly zero, the normal might not be reliable.
    pub normal: Vect,

    #[cfg(dim3)]
    /// The textures coordinates at the intersection point.  This is an `Option` because some shape
    /// do not support texture coordinates.
    pub uvs:    Option<Vect>
}

impl RayIntersection {
    #[cfg(dim3)]
    #[inline]
    /// Creates a new `RayIntersection`.
    pub fn new_with_uvs(toi: Scalar, normal: Vect, uvs: Option<Vect>) -> RayIntersection {
        RayIntersection {
            toi:    toi,
            normal: normal,
            uvs:    uvs
        }
    }

    #[cfg(not(dim3))]
    #[inline]
    /// Creates a new `RayIntersection`.
    pub fn new(toi: Scalar, normal: Vect) -> RayIntersection {
        RayIntersection {
            toi:    toi,
            normal: normal
        }
    }

    #[cfg(dim3)]
    #[inline]
    /// Creates a new `RayIntersection`.
    pub fn new(toi: Scalar, normal: Vect) -> RayIntersection {
        RayIntersection {
            toi:    toi,
            normal: normal,
            uvs:    None
        }
    }
}

/// Traits of objects which can be tested for intersection with a ray.
pub trait RayCast {
    /// Computes the time of impact between this geometry and a ray
    #[inline]
    fn toi_with_ray(&self, ray: &Ray, solid: bool) -> Option<Scalar> {
        self.toi_and_normal_with_ray(ray, solid).map(|inter| inter.toi)
    }

    /// Computes the intersection point between this geometry and a ray.
    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray, solid: bool) -> Option<RayIntersection>;

    /// Computes the intersection point and normal between this geometry and a ray.
    #[cfg(dim3)]
    #[inline]
    fn toi_and_normal_and_uv_with_ray(&self, ray: &Ray, solid: bool) -> Option<RayIntersection> {
        self.toi_and_normal_with_ray(ray, solid)
    }

    /// Tests whether a ray intersects this geometry.
    #[inline]
    fn intersects_ray(&self, ray: &Ray) -> bool {
        self.toi_with_ray(ray, true).is_some()
    }

    /// Computes the time of impact between this transform geometry and a ray.
    fn toi_with_transform_and_ray(&self, m: &Matrix, ray: &Ray, solid: bool) -> Option<Scalar> {
        let ls_ray = Ray::new(m.inv_transform(&ray.orig), m.inv_rotate(&ray.dir));

        self.toi_with_ray(&ls_ray, solid)
    }

    /// Computes the time of impact, and normal between this transformed geometry and a ray.
    #[inline]
    fn toi_and_normal_with_transform_and_ray(&self, m: &Matrix, ray: &Ray, solid: bool) -> Option<RayIntersection> {
        let ls_ray = Ray::new(m.inv_transform(&ray.orig), m.inv_rotate(&ray.dir));

        self.toi_and_normal_with_ray(&ls_ray, solid).map(|mut inter| {
            inter.normal = m.rotate(&inter.normal);

            inter
        })
    }

    /// Computes time of impact, normal, and texture coordinates (uv) between this transformed
    /// geometry and a ray.
    #[cfg(dim3)]
    #[inline]
    fn toi_and_normal_and_uv_with_transform_and_ray(&self, m: &Matrix, ray: &Ray, solid: bool) -> Option<RayIntersection> {
        let ls_ray = Ray::new(m.inv_transform(&ray.orig), m.inv_rotate(&ray.dir));

        self.toi_and_normal_and_uv_with_ray(&ls_ray, solid).map(|mut inter| {
            inter.normal = m.rotate(&inter.normal);

            inter
        })
    }

    /// Tests whether a ray intersects this transformed geometry.
    #[inline]
    fn intersects_with_transform_and_ray(&self, m: &Matrix, ray: &Ray) -> bool {
        self.toi_with_transform_and_ray(m, ray, true).is_some()
    }
}
