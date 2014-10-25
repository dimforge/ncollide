use std::num::Zero;
use na::{Pnt2, Transform, Rotate};
use na;
use ray::{Ray, LocalRayCast, RayCast, RayIntersection};
use ray;
use geom::{Mesh, MeshElement};
use math::{Scalar, Point, Vect};


impl<N, P, V, E> LocalRayCast<N, P, V> for Mesh<N, P, V, E>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          E: MeshElement<P> + LocalRayCast<N, P, V> {
    fn toi_with_ray(&self, ray: &Ray<P, V>, _: bool) -> Option<N> {
        self.bvt().cast_ray(
                ray,
                &mut |b, r| self.element_at(*b).toi_with_ray(r, true).map(|t| (t.clone(), t))
            ).map(|(_, res, _)| res)
    }

    fn toi_and_normal_with_ray(&self, ray: &Ray<P, V>, _: bool) -> Option<RayIntersection<N, V>> {
        self.bvt().cast_ray(
            ray,
            &mut |b, r| self.element_at(*b).toi_and_normal_with_ray(r, true).map(
                |inter| (inter.toi.clone(), inter))).map(
                    |(_, res, _)| res)
    }

    fn toi_and_normal_and_uv_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<RayIntersection<N, V>> {
        if self.uvs().is_none() || na::dim::<P>() != 3 {
            return self.toi_and_normal_with_ray(ray, solid);
        }

        let cast = self.bvt().cast_ray(
            ray,
            &mut |b, r| {
                let vs: &[P] = self.vertices().as_slice();
                let i        = *b * 3;
                let is       = self.indices().slice(i, i + 3);

                ray::triangle_ray_intersection(&vs[is[0]], &vs[is[1]], &vs[is[2]], r).map(|inter|
                    (inter.ref0().toi.clone(), inter))
            });

        match cast {
            None                   => None,
            Some((_, inter, best)) => {
                let toi = inter.ref0().toi;
                let n   = inter.ref0().normal.clone();
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
                        let n1 = &ns.deref()[is[0]];
                        let n2 = &ns.deref()[is[1]];
                        let n3 = &ns.deref()[is[2]];

                        let mut n123 = *n1 * uv.x + *n2 * uv.y + *n3 * uv.z;

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

impl<N, P, V, E, M> RayCast<N, P, V, M> for Mesh<N, P, V, E>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> + Rotate<V>,
          E: MeshElement<P> + LocalRayCast<N, P, V> {
}
