use nalgebra::traits::vector::Vec;

/// A Ray.
#[deriving(ToStr)]
pub struct Ray<V> {
    /// Starting point of the ray.
    orig: V,
    /// Direction of the ray.
    dir:  V
}

impl<V> Ray<V> {
    /// Creates a new ray starting from `orig` and with the direction `dir`. `dir` must be
    /// normalized.
    pub fn new(orig: V, dir: V) -> Ray<V> {
        Ray {
            orig: orig,
            dir:  dir
        }
    }
}

/// Traits of objects which can be test for intersection with a ray.
pub trait RayCast<N, V: Vec<N>, M> {
    /// Computes the time of impact between this geometry and a ray
    fn toi_with_ray(&self, m: &M, ray: &Ray<V>) -> Option<N>;

    /// Computes the intersection point between this geometry and a ray.
    #[inline]
    fn intersection_with_ray(&self, m: &M, ray: &Ray<V>) -> Option<V> {
        self.toi_with_ray(m, ray).map(|t| ray.orig + ray.dir * *t)
    }

    /// Tests whether a ray intersects this geometry.
    #[inline]
    fn intersects_ray(&self, m: &M, ray: &Ray<V>) -> bool {
        self.toi_with_ray(m, ray).is_some()
    }
}
