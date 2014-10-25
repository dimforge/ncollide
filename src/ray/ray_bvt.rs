use na::{Transform, Rotate};
use ray::{Ray, LocalRayCast, RayCast, RayIntersection};
use partitioning::BVT;
use math::Scalar;


impl<N, P, V, B, BV> LocalRayCast<N, P, V> for BVT<B, BV>
    where N: Scalar,
          B:  LocalRayCast<N, P, V>,
          BV: LocalRayCast<N, P, V> {
    #[inline]
    fn toi_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<N> {
        self.cast_ray(
            ray,
            &mut |b, r| b.toi_with_ray(r, solid).map(
                |t| (t.clone(), t))).map(
                    |(_, res, _)| res)
    }

    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<RayIntersection<N, V>> {
        self.cast_ray(
            ray,
            &mut |b, r| b.toi_and_normal_with_ray(r, solid).map(
                |inter| (inter.toi.clone(), inter))).map(
                    |(_, res, _)| res)
    }

    #[inline]
    fn toi_and_normal_and_uv_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<RayIntersection<N, V>> {
        self.cast_ray(
            ray,
            &mut |b, r| b.toi_and_normal_and_uv_with_ray(r, solid).map(
                |inter| (inter.toi.clone(), inter))).map(
                    |(_, res, _)| res)
    }

    // FIXME: optimize insersect_ray ?
}

impl<N, P, V, M, B, BV> RayCast<N, P, V, M> for BVT<B, BV>
    where N: Scalar,
          B:  LocalRayCast<N, P, V>,
          BV: LocalRayCast<N, P, V>,
          M:  Transform<P> + Rotate<V> {
}
