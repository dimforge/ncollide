use ray::{Ray, RayCast, RayIntersection};
use geom::Mesh;
use math::Scalar;

#[cfg(dim3)]
use ray;
#[cfg(dim3)]
use std::num::Zero;
#[cfg(dim3)]
use nalgebra::na::{Vec3, Norm};
#[cfg(dim3)]
use nalgebra::na;
#[cfg(dim3)]
use math::Vect;


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

    #[cfg(dim3)]
    fn toi_and_normal_and_uv_with_ray(&self, ray: &Ray, _: bool) -> Option<RayIntersection> {
        if !self.margin().is_zero() || self.uvs().is_none() {
            return self.toi_and_normal_with_ray(ray, true);
        }

        let cast = self.bvt().cast_ray(
            ray,
            &mut |b, r| {
                let vs: &[Vect] = self.vertices().as_slice();
                let i        = *b * 3;
                let is       = self.indices().slice(i, i + 3);

                ray::triangle_ray_intersection(&vs[is[0]], &vs[is[1]], &vs[is[2]], r).map(|inter|
                    (inter.toi.clone(), inter)
            )});

        match cast {
            None                   => None,
            Some((_, inter, best)) => {
                let toi = inter.toi;
                let n   = inter.normal;
                let uv  = inter.uvs.unwrap();

                let ibest = *best * 3;
                let is    = self.indices().slice(ibest, ibest + 3);
                let uvs   = self.uvs().as_ref().unwrap();

                let (x1, y1, z1) = uvs.get(is[0]).clone();
                let (x2, y2, z2) = uvs.get(is[1]).clone();
                let (x3, y3, z3) = uvs.get(is[2]).clone();

                let uvx = x1 * uv.x + x2 * uv.y + x3 * uv.z;
                let uvy = y1 * uv.x + y2 * uv.y + y3 * uv.z;
                let uvz = z1 * uv.x + z2 * uv.y + z3 * uv.z;

                // XXX: this interpolation should be done on the two other ray cast too!
                match *self.normals() {
                    None         => {
                        Some(RayIntersection::new_with_uvs(toi, n, Some(Vec3::new(uvx, uvy, uvz))))
                    },
                    Some(ref ns) => {
                        let n1 = ns.get(is[0]);
                        let n2 = ns.get(is[1]);
                        let n3 = ns.get(is[2]);

                        let mut n123 = n1 * uv.x + n2 * uv.y + n3 * uv.z;

                        if n123.normalize().is_zero() {
                            Some(RayIntersection::new_with_uvs(toi, n, Some(Vec3::new(uvx, uvy, uvz))))
                        }
                        else {
                            if na::dot(&n123, &ray.dir) > na::zero() {
                                Some(RayIntersection::new_with_uvs(toi, -n123, Some(Vec3::new(uvx, uvy, uvz))))
                            }
                            else {
                                Some(RayIntersection::new_with_uvs(toi, n123, Some(Vec3::new(uvx, uvy, uvz))))
                            }
                        }
                    }
                }
            }
        }
    }
}
