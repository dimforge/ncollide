use ray::{Ray, RayCast, RayIntersection};
use partitioning::BVT;
use math::Scalar;

impl<B: RayCast, BV: RayCast> RayCast for BVT<B, BV> {
    #[inline]
    fn toi_with_ray(&self, ray: &Ray, solid: bool) -> Option<Scalar> {
        self.cast_ray(
            ray,
            &mut |b, r| b.toi_with_ray(r, solid).map(
                |t| (t.clone(), t))).map(
                    |(_, res, _)| res)
    }

    #[inline]
    fn toi_and_normal_with_ray(&self, ray: &Ray, solid: bool) -> Option<RayIntersection> {
        self.cast_ray(
            ray,
            &mut |b, r| b.toi_and_normal_with_ray(r, solid).map(
                |inter| (inter.toi.clone(), inter))).map(
                    |(_, res, _)| res)
    }

    // #[cfg(feature = "3d")]
    #[inline]
    fn toi_and_normal_and_uv_with_ray(&self, ray: &Ray, solid: bool) -> Option<RayIntersection> {
        self.cast_ray(
            ray,
            &mut |b, r| b.toi_and_normal_and_uv_with_ray(r, solid).map(
                |inter| (inter.toi.clone(), inter))).map(
                    |(_, res, _)| res)
    }

    // FIXME: optimize insersect_ray ?
}
