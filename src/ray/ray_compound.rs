use nalgebra::traits::transformation::Transform;
use nalgebra::traits::rotation::Rotate;
use nalgebra::traits::vector::AlgebraicVecExt;
use ray::ray::{Ray, RayCast};
use geom::compound::CompoundAABB;


impl<N: Num + Bounded + Orderable + Primitive + Algebraic + ToStr,
     V: 'static + AlgebraicVecExt<N> + ToStr,
     M: Transform<V> + Rotate<V>,
     S: RayCast<N, V, M>>
RayCast<N, V, M> for CompoundAABB<N, V, M, S> {
    fn toi_with_ray(&self, m: &M, ray: &Ray<V>) -> Option<N> {

        let ls_ray = Ray::new(m.inv_transform(&ray.orig), m.inv_rotate(&ray.dir));

        let mut interferences = ~[];
        self.dbvt().interferences_with_ray(&ls_ray, &mut interferences);

        // compute the minimum toi
        let mut toi = Bounded::max_value::<N>();
        let shapes = self.shapes();

        for i in interferences.iter() {
            let (ref objm, ref obj) = shapes[i.object];

            match obj.toi_with_ray(objm, &ls_ray) {
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
