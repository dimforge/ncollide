use ray::{Ray, RayCast, RayIntersection};
use partitioning::BVT;
use math::N;

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
    fn toi_and_normal_with_ray(&self, ray: &Ray) -> Option<RayIntersection> {
        self.cast_ray(
            ray,
            &|b, r| b.toi_and_normal_with_ray(r).map(
                |inter| (inter.toi.clone(), inter))).map(
                    |(_, res, _)| res)
    }

    #[cfg(dim3)]
    #[inline]
    fn toi_and_normal_and_uv_with_ray(&self, ray: &Ray) -> Option<RayIntersection> {
        self.cast_ray(
            ray,
            &|b, r| b.toi_and_normal_and_uv_with_ray(r).map(
                |inter| (inter.toi.clone(), inter))).map(
                    |(_, res, _)| res)
    }

    // FIXME: optimize insersect_ray ?
}
