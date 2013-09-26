use nalgebra::vec::AlgebraicVecExt;
use nalgebra::mat::{Rotate, Transform};
use ray::{Ray, RayCast, RayCastWithTransform};
use geom::CompoundAABB;
use partitioning::bvt_visitor::RayInterferencesCollector;


impl<N: Num + Bounded + Orderable + Primitive + Algebraic + ToStr,
     V: 'static + AlgebraicVecExt<N> + ToStr,
     M: Transform<V> + Rotate<V>,
     S: RayCastWithTransform<N, V, M>>
RayCast<N, V> for CompoundAABB<N, V, M, S> {
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
}

impl<N: Num + Bounded + Orderable + Primitive + Algebraic + ToStr,
     V: 'static + AlgebraicVecExt<N> + ToStr,
     M: Transform<V> + Rotate<V>,
     S: RayCastWithTransform<N, V, M>>
RayCastWithTransform<N, V, M> for CompoundAABB<N, V, M, S> { }
