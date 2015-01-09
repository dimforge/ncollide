use na::Translate;
use na;
use bounding_volume::{HasBoundingSphere, BoundingSphere};
use shape::Capsule;
use math::{Scalar, Point};


#[old_impl_check]
impl<N, P, V, M> HasBoundingSphere<N, P, M> for Capsule<N>
    where N: Scalar,
          P: Point<N, V>,
          M: Translate<P> {
    #[inline]
    fn bounding_sphere(&self, m: &M) -> BoundingSphere<N, P> {
        let center = m.translate(&na::orig());
        let radius = self.radius() + self.half_height();

        BoundingSphere::new(center, radius)
    }
}
