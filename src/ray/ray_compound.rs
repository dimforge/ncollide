use nalgebra::traits::transformation::Transform;
use nalgebra::traits::rotation::Rotate;
use nalgebra::traits::vector::AlgebraicVecExt;
use ray::ray::{Ray, RayCast, RayCastWithTransform};
use bounding_volume::aabb::AABB;
use geom::compound::CompoundAABB;
use partitioning::bvt_visitor::RayInterferencesCollector;
use partitioning::dbvt::DBVTLeaf;


impl<N: Num + Bounded + Orderable + Primitive + Algebraic + ToStr,
     V: 'static + AlgebraicVecExt<N> + ToStr,
     M: Transform<V> + Rotate<V>,
     S: RayCastWithTransform<N, V, M>>
RayCast<N, V> for CompoundAABB<N, V, M, S> {
    fn toi_with_ray(&self, ray: &Ray<V>) -> Option<N> {
        // FIXME: why cant the array type be infered here?
        let mut interferences: ~[@mut DBVTLeaf<V, uint, AABB<N, V>>] = ~[];

        {
            let mut visitor = RayInterferencesCollector::new(ray, &mut interferences);
            self.dbvt().visit(&mut visitor);
        }

        // compute the minimum toi
        let mut toi = Bounded::max_value::<N>();
        let shapes = self.shapes();

        for i in interferences.iter() {
            let (ref objm, ref obj) = shapes[i.object];

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
RayCastWithTransform<N, V, M> for CompoundAABB<N, V, M, S>;
