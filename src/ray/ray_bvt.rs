use ray::{Ray, RayCast, RayCastWithTransform};
use partitioning::BVT;
use math::{N, V};

impl<B: RayCast, BV: RayCast> RayCast for BVT<B, BV> {
    #[inline]
    fn toi_with_ray(&self, ray: &Ray) -> Option<N> {
        self.cast_ray(
            ray,
            &|b, r| b.toi_with_ray(r).map(
                |t| (t.clone(), t))).map(
                    |(_, res, _)| res)
    }

    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray) -> Option<(N, V)> {
        self.cast_ray(
            ray,
            &|b, r| b.toi_and_normal_with_ray(r).map(
                |(t, n)| (t.clone(), (t, n)))).map(
                    |(_, res, _)| res)
    }

    #[cfg(dim3)]
    #[inline]
    fn toi_and_normal_and_uv_with_ray(&self, ray: &Ray) -> Option<(N, V, Option<(N, N, N)>)> {
        self.cast_ray(
            ray,
            &|b, r| b.toi_and_normal_and_uv_with_ray(r).map(
                |(t, n, uv)| (t.clone(), (t, n, uv)))).map(
                    |(_, res, _)| res)
    }

    // FIXME: optimize insersect_ray ?
}

impl<B: RayCast, BV: RayCast> RayCastWithTransform for BVT<B, BV> { }
