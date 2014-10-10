use std::num::Zero;
use ray::{Ray, RayCast, RayIntersection};
use geom::Mesh;
use math::Scalar;

// #[cfg(feature = "3d")]
use na::Pnt2;
// #[cfg(feature = "3d")]
use ray;
// #[cfg(feature = "3d")]
use na::Norm;
// #[cfg(feature = "3d")]
use na;
// #[cfg(feature = "3d")]
use math::Point;


impl RayCast for Mesh {
    fn toi_with_ray(&self, ray: &Ray, _: bool) -> Option<Scalar> {
        self.bvt().cast_ray(
                ray,
                &mut |b, r| self.element_at(*b).toi_with_ray(r, true).map(|t| (t.clone(), t))
            ).map(|(_, res, _)| res)
    }

    fn toi_and_normal_with_ray(&self, ray: &Ray, _: bool) -> Option<RayIntersection> {
        self.bvt().cast_ray(
            ray,
            &mut |b, r| self.element_at(*b).toi_and_normal_with_ray(r, true).map(
                |inter| (inter.toi.clone(), inter))).map(
                    |(_, res, _)| res)
    }

    // #[cfg(feature = "3d")]
    fn toi_and_normal_and_uv_with_ray(&self, ray: &Ray, solid: bool) -> Option<RayIntersection> {
        if self.uvs().is_none() {
            return self.toi_and_normal_with_ray(ray, solid);
        }

        let cast = self.bvt().cast_ray(
            ray,
            &mut |b, r| {
                let vs: &[Point] = self.vertices().as_slice();
                let i            = *b * 3;
                let is           = self.indices().slice(i, i + 3);

                ray::triangle_ray_intersection(&vs[is[0]], &vs[is[1]], &vs[is[2]], r).map(|inter|
                    (inter.ref0().toi.clone(), inter))
            });

        match cast {
            None                   => None,
            Some((_, inter, best)) => {
                let toi = inter.ref0().toi;
                let n   = inter.ref0().normal;
                let uv  = inter.val1(); // barycentric coordinates to compute the exact uvs.

                let ibest = *best * 3;
                let is    = self.indices().slice(ibest, ibest + 3);
                let uvs   = self.uvs().as_ref().unwrap();

                let uv1 = uvs.deref()[is[0]];
                let uv2 = uvs.deref()[is[1]];
                let uv3 = uvs.deref()[is[2]];

                let uvx = uv1.x * uv.x + uv2.x * uv.y + uv3.x * uv.z;
                let uvy = uv1.y * uv.x + uv2.y * uv.y + uv3.y * uv.z;

                // XXX: this interpolation should be done on the two other ray cast too!
                match *self.normals() {
                    None         => {
                        Some(RayIntersection::new_with_uvs(toi, n, Some(Pnt2::new(uvx, uvy))))
                    },
                    Some(ref ns) => {
                        let n1 = ns.deref()[is[0]];
                        let n2 = ns.deref()[is[1]];
                        let n3 = ns.deref()[is[2]];

                        let mut n123 = n1 * uv.x + n2 * uv.y + n3 * uv.z;

                        if n123.normalize().is_zero() {
                            Some(RayIntersection::new_with_uvs(toi, n, Some(Pnt2::new(uvx, uvy))))
                        }
                        else {
                            if na::dot(&n123, &ray.dir) > na::zero() {
                                Some(RayIntersection::new_with_uvs(toi, -n123, Some(Pnt2::new(uvx, uvy))))
                            }
                            else {
                                Some(RayIntersection::new_with_uvs(toi, n123, Some(Pnt2::new(uvx, uvy))))
                            }
                        }
                    }
                }
            }
        }
    }
}
