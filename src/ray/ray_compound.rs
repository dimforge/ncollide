use std::num::Zero;
use nalgebra::na::{AlgebraicVecExt, Rotate, Transform};
use ray::{Ray, RayCast, RayCastWithTransform};
use geom::CompoundAABB;
use partitioning::bvt_visitor::RayInterferencesCollector;


impl<N: Num + Bounded + Orderable + Primitive + Algebraic,
     V: 'static + AlgebraicVecExt<N>,
     M: Transform<V> + Rotate<V>,
     S: RayCastWithTransform<N, V, M>>
RayCast<N, V> for CompoundAABB<N, V, M, S> {
    // FIXME: find a way to refactore those two methods

    fn toi_with_ray(&self, ray: &Ray<V>) -> Option<N> {
        // FIXME: why cant the array type be infered here?
        let mut interferences: ~[uint] = ~[];

        {
            let mut visitor = RayInterferencesCollector::new(ray, &mut interferences);
            self.bvt().visit(&mut visitor);
        }

        // compute the minimum toi
        let mut toi: N = Bounded::max_value();
        let shapes = self.shapes();

        for i in interferences.iter() {
            let (ref objm, ref obj) = shapes[*i];

            match obj.toi_with_transform_and_ray(objm, ray) {
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
        // FIXME: why cant the array type be infered here?
        let mut interferences: ~[uint] = ~[];

        {
            let mut visitor = RayInterferencesCollector::new(ray, &mut interferences);
            self.bvt().visit(&mut visitor);
        }

        // compute the minimum toi
        let mut toi: (N, V) = (Bounded::max_value(), Zero::zero());
        let shapes = self.shapes();

        for i in interferences.iter() {
            let (ref objm, ref obj) = shapes[*i];

            match obj.toi_and_normal_with_transform_and_ray(objm, ray) {
                None    => { },
                Some(t) => {
                    if *toi.first_ref() < *t.first_ref() {
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
}

impl<N: Num + Bounded + Orderable + Primitive + Algebraic,
     V: 'static + AlgebraicVecExt<N>,
     M: Transform<V> + Rotate<V>,
     S: RayCastWithTransform<N, V, M>>
RayCastWithTransform<N, V, M> for CompoundAABB<N, V, M, S> { }
