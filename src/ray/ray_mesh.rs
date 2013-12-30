use std::num::{Zero, One};
use nalgebra::na::{AlgebraicVecExt, Rotate, Transform, Cast, Translation, AbsoluteRotate};
use nalgebra::na;
use partitioning::bvt_visitor::RayInterferencesCollector;
use ray::{Ray, RayCast, RayCastWithTransform};
use ray;
use geom::{Geom, Mesh, MeshElement};


impl<N:  Freeze + Send + Num + Bounded + Orderable + Primitive + Algebraic + Cast<f32> + Signed,
     V:  Freeze + Send + AlgebraicVecExt<N> + Clone,
     M:  Transform<V> + Rotate<V> + Clone + Mul<M, M> + Translation<V> + AbsoluteRotate<V> + One,
     II: Zero + Add<II, II>,
     E:  Geom<N, V, M, II> + MeshElement<N, V>>
RayCast<N, V> for Mesh<N, V, M, II, E> {
    fn toi_with_ray(&self, ray: &Ray<V>) -> Option<N> {
        let mut interferences: ~[uint] = ~[];

        {
            let mut visitor = RayInterferencesCollector::new(ray, &mut interferences);
            self.bvt().visit(&mut visitor);
        }

        // compute the minimum toi
        let mut toi: N = Bounded::max_value();

        for i in interferences.iter() {
            let element = self.element_at(*i);

            match element.toi_with_ray(ray) {
                None        => { },
                Some(ref t) => toi = toi.min(t)
            }
        }

        if toi == Bounded::max_value() {
            None
        }
        else {
            Some(toi)
        }
    }

    fn toi_and_normal_with_ray(&self, ray: &Ray<V>) -> Option<(N, V)> {
        let mut interferences: ~[uint] = ~[];

        {
            let mut visitor = RayInterferencesCollector::new(ray, &mut interferences);
            self.bvt().visit(&mut visitor);
        }

        // compute the minimum toi
        let mut toi: (N, V) = (Bounded::max_value(), na::zero());

        for i in interferences.iter() {
            let element = self.element_at(*i);

            match element.toi_and_normal_with_ray(ray) {
                None    => { },
                Some(t) => {
                    if *t.first_ref() < *toi.first_ref() {
                        toi = t
                    }
                }
            }
        }

        if *toi.first_ref() == Bounded::max_value() {
            None
        }
        else {
            Some(toi)
        }
    }

    fn toi_and_normal_and_uv_with_ray(&self, ray: &Ray<V>) -> Option<(N, V, Option<(N, N, N)>)> {
        if na::dim::<V>() != 3 || !self.margin().is_zero() || self.uvs().is_none() {
            return self.toi_and_normal_with_ray(ray).map(|(toi, n)| (toi, n, None));
        }

        let mut interferences: ~[uint] = ~[];

        {
            let mut visitor = RayInterferencesCollector::new(ray, &mut interferences);
            self.bvt().visit(&mut visitor);
        }

        // compute the minimum toi
        let mut best = 0;
        let mut toi: (N, V, (N, N, N)) = (Bounded::max_value(), na::zero(), (na::zero(), na::zero(), na::zero()));

        for i in interferences.iter() {
            let vs: &[V] = *self.vertices().get();
            let i        = i * MeshElement::nvertices(None::<E>);
            let is       = self.indices().get().slice(i, i + MeshElement::nvertices(None::<E>));

            match ray::triangle_ray_intersection(&vs[is[0]], &vs[is[1]], &vs[is[2]], ray) {
                None    => { },
                Some(t) => {
                    if *t.n0_ref() < *toi.n0_ref() {
                        best = i;
                        toi  = t
                    }
                }
            }
        }

        if *toi.n0_ref() == Bounded::max_value() {
            None
        }
        else {
            let (toi, n, (u, v, w)) = toi;
            let is  = self.indices().get().slice(best, best + MeshElement::nvertices(None::<E>));
            let uvs = self.uvs().as_ref().unwrap().get();

            let (x1, y1, z1) = uvs[is[0]].clone();
            let (x2, y2, z2) = uvs[is[1]].clone();
            let (x3, y3, z3) = uvs[is[2]].clone();

            let uvx = x1 * u + x2 * v + x3 * w;
            let uvy = y1 * u + y2 * v + y3 * w;
            let uvz = z1 * u + z2 * v + z3 * w;

            // XXX: this interpolation should be done on the two other ray cast too!
            match *self.normals() {
                None         => Some((toi, n, Some((uvx, uvy, uvz)))),
                Some(ref ns) => {
                    let ns = ns.get();

                    let n1 = &ns[is[0]];
                    let n2 = &ns[is[1]];
                    let n3 = &ns[is[2]];

                    let n  = na::normalize(&(n1 * u + n2 * v + n3 * w));

                    if na::dot(&n, &ray.dir) > na::zero() {
                        Some((toi, -n, Some((uvx, uvy, uvz))))
                    }
                    else {
                        Some((toi, n, Some((uvx, uvy, uvz))))
                    }
                }
            }
        }
    }
}

impl<N:  Num + Bounded + Orderable + Primitive + Algebraic + Cast<f32> + Signed,
     V:  AlgebraicVecExt<N> + Clone,
     M:  Transform<V> + Rotate<V> + Clone + Mul<M, M> + Translation<V> + AbsoluteRotate<V> + One,
     II: Zero + Add<II, II>,
     E:  Geom<N, V, M, II> + MeshElement<N, V>>
RayCastWithTransform<N, V, M> for Mesh<N, V, M, II, E> { }
