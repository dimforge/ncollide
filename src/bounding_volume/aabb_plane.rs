use na::Bounded;
use na;
use bounding_volume::{HasAABB, AABB};
use shape::Plane;
use math::{Scalar, Point};


impl<N, P, V, M> HasAABB<P, M> for Plane<V>
    where N: Scalar,
          P: Point<N, V> {
    #[inline]
    fn aabb(&self, _: &M) -> AABB<P> {
        // we divide by 2.0  so that we can still make some operations with it (like loosening)
        // without breaking the box.
        let max: P = Bounded::max_value();
        let half: N = na::cast(0.5f64);
        let half_max = max * half;

        AABB::new(-half_max, half_max)
    }
}
