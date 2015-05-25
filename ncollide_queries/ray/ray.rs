//! Traits and structure needed to cast rays.

use na::{Rotate, Transform, Pnt2};
use math::{Point, Vect};

/// A Ray.
#[derive(Debug, RustcEncodable, RustcDecodable, Clone)]
pub struct Ray<P: Point> {
    /// Starting point of the ray.
    pub orig: P,
    /// Direction of the ray.
    pub dir:  P::Vect
}

impl<P: Point> Ray<P> {
    /// Creates a new ray starting from `orig` and with the direction `dir`. `dir` must be
    /// normalized.
    pub fn new(orig: P, dir: P::Vect) -> Ray<P> {
        Ray {
            orig: orig,
            dir:  dir
        }
    }
}

/// Structure containing the result of a successful ray cast.
pub struct RayIntersection<V: Vect> {
    /// The time of impact of the ray with the object.  The exact contact point can be computed
    /// with: `orig + dir * toi` where `orig` is the origin of the ray; `dir` is its direction and
    /// `toi` is the value of this field.
    pub toi:    V::Scalar,

    /// The normal at the intersection point.
    ///
    /// If the `toi` is exactly zero, the normal might not be reliable.
    pub normal: V,

    /// The textures coordinates at the intersection point.  This is an `Option` because some shape
    /// do not support texture coordinates.
    pub uvs:    Option<Pnt2<V::Scalar>>
}

impl<V: Vect> RayIntersection<V> {
    #[inline]
    /// Creates a new `RayIntersection`.
    pub fn new_with_uvs(toi: V::Scalar, normal: V, uvs: Option<Pnt2<V::Scalar>>) -> RayIntersection<V> {
        RayIntersection {
            toi:    toi,
            normal: normal,
            uvs:    uvs
        }
    }

    #[inline]
    /// Creates a new `RayIntersection`.
    pub fn new(toi: V::Scalar, normal: V) -> RayIntersection<V> {
        RayIntersection {
            toi:    toi,
            normal: normal,
            uvs:    None
        }
    }
}

/// Traits of objects which can be tested for intersection with a ray.
pub trait LocalRayCast<P: Point> {
    /// Computes the time of impact between this shape and a ray
    #[inline]
    fn toi_with_ray(&self, ray: &Ray<P>, solid: bool) -> Option<<P::Vect as Vect>::Scalar> {
        self.toi_and_normal_with_ray(ray, solid).map(|inter| inter.toi)
    }

    /// Computes the intersection point between this shape and a ray.
    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vect>>;

    /// Computes the intersection point and normal between this shape and a ray.
    #[inline]
    fn toi_and_normal_and_uv_with_ray(&self, ray: &Ray<P>, solid: bool) -> Option<RayIntersection<P::Vect>> {
        self.toi_and_normal_with_ray(ray, solid)
    }

    /// Tests whether a ray intersects this shape.
    #[inline]
    fn intersects_ray(&self, ray: &Ray<P>) -> bool {
        self.toi_with_ray(ray, true).is_some()
    }
}

// FIXME: replace this trait by free functions?
/// Traits of objects which can be transformed and tested for intersection with a ray.
pub trait RayCast<P: Point, M: Transform<P> + Rotate<P::Vect>>: LocalRayCast<P> {
    /// Computes the time of impact between this transform shape and a ray.
    fn toi_with_transform_and_ray(&self, m: &M, ray: &Ray<P>, solid: bool) -> Option<<P::Vect as Vect>::Scalar> {
        let ls_ray = Ray::new(m.inv_transform(&ray.orig), m.inv_rotate(&ray.dir));

        self.toi_with_ray(&ls_ray, solid)
    }

    /// Computes the time of impact, and normal between this transformed shape and a ray.
    #[inline]
    fn toi_and_normal_with_transform_and_ray(&self, m: &M, ray: &Ray<P>, solid: bool)
                                             -> Option<RayIntersection<P::Vect>> {
        let ls_ray = Ray::new(m.inv_transform(&ray.orig), m.inv_rotate(&ray.dir));

        self.toi_and_normal_with_ray(&ls_ray, solid).map(|mut inter| {
            inter.normal = m.rotate(&inter.normal);

            inter
        })
    }

    /// Computes time of impact, normal, and texture coordinates (uv) between this transformed
    /// shape and a ray.
    #[inline]
    fn toi_and_normal_and_uv_with_transform_and_ray(&self, m: &M, ray: &Ray<P>, solid: bool)
                                                    -> Option<RayIntersection<P::Vect>> {
        let ls_ray = Ray::new(m.inv_transform(&ray.orig), m.inv_rotate(&ray.dir));

        self.toi_and_normal_and_uv_with_ray(&ls_ray, solid).map(|mut inter| {
            inter.normal = m.rotate(&inter.normal);

            inter
        })
    }

    /// Tests whether a ray intersects this transformed shape.
    #[inline]
    fn intersects_with_transform_and_ray(&self, m: &M, ray: &Ray<P>) -> bool {
        self.toi_with_transform_and_ray(m, ray, true).is_some()
    }
}
