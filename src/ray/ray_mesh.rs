use ray::{Ray, RayCast, RayCastWithTransform};
use geom::Mesh;
use math::{N, V};

#[cfg(dim3)]
use ray;
#[cfg(dim3)]
use std::num::Zero;
#[cfg(dim3)]
use nalgebra::na::Norm;
#[cfg(dim3)]
use nalgebra::na;


impl RayCast for Mesh {
    fn toi_with_ray(&self, ray: &Ray) -> Option<N> {
        self.bvt().cast_ray(
                ray,
                &|b, r| self.element_at(*b).toi_with_ray(r).map(|t| (t.clone(), t))
            ).map(|(_, res, _)| res)
    }

    fn toi_and_normal_with_ray(&self, ray: &Ray) -> Option<(N, V)> {
        self.bvt().cast_ray(
            ray,
            &|b, r| self.element_at(*b).toi_and_normal_with_ray(r).map(
                |(t, n)| (t.clone(), (t, n)))).map(
                    |(_, res, _)| res)
    }

    #[cfg(dim3)]
    fn toi_and_normal_and_uv_with_ray(&self, ray: &Ray) -> Option<(N, V, Option<(N, N, N)>)> {
        if !self.margin().is_zero() || self.uvs().is_none() {
            return self.toi_and_normal_with_ray(ray).map(|(toi, n)| (toi, n, None));
        }

        let cast = self.bvt().cast_ray(
            ray,
            &|b, r| {
                let vs: &[V] = *self.vertices().get();
                let i        = *b * 3;
                let is       = self.indices().get().slice(i, i + 3);

                ray::triangle_ray_intersection(&vs[is[0]], &vs[is[1]], &vs[is[2]], r).map(|t|
                    (t.n0_ref().clone(), t)
            )});

        match cast {
            None                   => None,
            Some((_, inter, best)) => {
                let (toi, n, (u, v, w)) = inter;
                let ibest = *best * 3;
                let is    = self.indices().get().slice(ibest, ibest + 3);
                let uvs   = self.uvs().as_ref().unwrap().get();

                let (x1, y1, z1) = uvs[is[0]].clone();
                let (x2, y2, z2) = uvs[is[1]].clone();
                let (x3, y3, z3) = uvs[is[2]].clone();

                let uvx = x1 * u + x2 * v + x3 * w;
                let uvy = y1 * u + y2 * v + y3 * w;
                let uvz = z1 * u + z2 * v + z3 * w;

                // XXX: this interpolation should be done on the two other ray cast too!
                match *self.normals() {
                    None         => {
                        if na::norm(&n).is_zero() {
                            println!("Computed normal is zero: {:?}", n);
                        }
                        Some((toi, n, Some((uvx, uvy, uvz))))
                    },
                    Some(ref ns) => {
                        let ns = ns.get();

                        let n1 = &ns[is[0]];
                        let n2 = &ns[is[1]];
                        let n3 = &ns[is[2]];

                        let mut n123 = n1 * u + n2 * v + n3 * w;

                        if na::norm(&n123).is_zero() {
                            println!("Interpolated normal is zero: {:?}", n123);
                        }
                        if na::norm(&n).is_zero() {
                            println!("Computed normal is zero: {:?}", n);
                        }

                        if n123.normalize().is_zero() {
                            Some((toi, n, Some((uvx, uvy, uvz))))
                        }
                        else {
                            if na::dot(&n123, &ray.dir) > na::zero() {
                                Some((toi, -n123, Some((uvx, uvy, uvz))))
                            }
                            else {
                                Some((toi, n123, Some((uvx, uvy, uvz))))
                            }
                        }
                    }
                }
            }
        }
    }
}

impl RayCastWithTransform for Mesh { }
