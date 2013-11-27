use nalgebra::na::{Vec, Rotate, Transform};

/// A Ray.
#[deriving(ToStr, Encodable, Decodable)]
pub struct Ray<V> {
    /// Starting point of the ray.
    orig: V,
    /// Direction of the ray.
    dir:  V
}

impl<N, V: Vec<N>> Ray<V> {
    /// Creates a new ray starting from `orig` and with the direction `dir`. `dir` must be
    /// normalized.
    pub fn new(orig: V, dir: V) -> Ray<V> {
        Ray {
            orig: orig,
            dir:  dir
        }
    }
}

/// Traits of transformable objects which can be tested for intersection with a ray.
pub trait RayCastWithTransform<N, V: Vec<N>, M: Rotate<V> + Transform<V>> : RayCast<N, V> {
    /// Computes the time of impact between this transform geometry and a ray.
    fn toi_with_transform_and_ray(&self, m: &M, ray: &Ray<V>) -> Option<N> {
        let ls_ray = Ray::new(m.inv_transform(&ray.orig), m.inv_rotate(&ray.dir));

        self.toi_with_ray(&ls_ray)
    }

    /// Computes the time of impact, and normal between this transformed geometry and a ray.
    #[inline]
    fn toi_and_normal_with_transform_and_ray(&self, m: &M, ray: &Ray<V>) -> Option<(N, V)> {
        let ls_ray = Ray::new(m.inv_transform(&ray.orig), m.inv_rotate(&ray.dir));

        self.toi_and_normal_with_ray(&ls_ray).map(|(n, d)| {
            (n, m.rotate(&d))
        })
    }

    /// Tests whether a ray intersects this transformed geometry.
    #[inline]
    fn intersects_with_transform_and_ray(&self, m: &M, ray: &Ray<V>) -> bool {
        self.toi_with_transform_and_ray(m, ray).is_some()
    }
}

/// Traits of objects which can be tested for intersection with a ray.
pub trait RayCast<N, V: Vec<N>> {
    /// Computes the time of impact between this geometry and a ray
    #[inline]
    fn toi_with_ray(&self, ray: &Ray<V>) -> Option<N> {
        self.toi_and_normal_with_ray(ray).map(|(n, _)| n)
    }

    /// Computes the intersection point between this geometry and a ray.
    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray<V>) -> Option<(N, V)>;

    /// Tests whether a ray intersects this geometry.
    #[inline]
    fn intersects_ray(&self, ray: &Ray<V>) -> bool {
        self.toi_with_ray(ray).is_some()
    }
}
