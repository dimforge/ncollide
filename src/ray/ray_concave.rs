use std::num::{Zero, One};
use nalgebra::na::{Vec, AlgebraicVecExt, Rotate, Transform, Cast, Translation, AbsoluteRotate};
use nalgebra::na;
use volumetric::InertiaTensor;
use ray::{Ray, RayCast, RayCastWithTransform};
use geom::{Geom, ConcaveGeom, Compound, Mesh, MeshElement};

fn toi_with_ray_on_concave_geom<N: Orderable + Ord + Eq + Bounded,
                                V: Vec<N>,
                                M: Rotate<V> + Transform<V>,
                                II,
                                G: ConcaveGeom<N, V, M, II>>(
                                g:   &G,
                                ray: &Ray<V>)
                                -> Option<N> {
    // FIXME: why cant the array type be infered here?
    // XXX: avoid this allocation
    let mut interferences: ~[uint] = ~[];

    g.approx_interferences_with_ray(ray, &mut interferences);

    // compute the minimum toi
    let mut toi: N = Bounded::max_value();

    for i in interferences.iter() {
        g.map_part_at(*i, |objm, obj|
            match obj.toi_with_transform_and_ray(objm, ray) {
                None        => { },
                Some(ref t) => toi = toi.min(t)
            }
        );
    }

    if toi == Bounded::max_value() {
        None
    }
    else {
        Some(toi)
    }
}

fn toi_and_normal_with_ray_on_concave_geom<N: Orderable + Ord + Eq + Bounded,
                                           V: Vec<N>,
                                           M: Rotate<V> + Transform<V>,
                                           II,
                                           G: ConcaveGeom<N, V, M, II>>(
                                           g:   &G,
                                           ray: &Ray<V>)
                                           -> Option<(N, V)> {
    // FIXME: why cant the array type be infered here?
    let mut interferences: ~[uint] = ~[];

    g.approx_interferences_with_ray(ray, &mut interferences);

    // compute the minimum toi
    let mut toi: (N, V) = (Bounded::max_value(), na::zero());

    for i in interferences.iter() {
        g.map_part_at(*i, |objm, obj|
            match obj.toi_and_normal_with_transform_and_ray(objm, ray) {
                None    => { },
                Some(t) => {
                    if *t.first_ref() < *toi.first_ref() {
                        toi = t
                    }
                }
            }
        );
    }

    if *toi.first_ref() == Bounded::max_value() {
        None
    }
    else {
        Some(toi)
    }
}

impl<N:  Clone + Zero + Num + Primitive + Orderable + Cast<f32> + Algebraic,
     LV: Clone + Zero + AlgebraicVecExt<N>,
     AV,
     M:  Clone + Mul<M, M> + Translation<LV> + AbsoluteRotate<LV> + Transform<LV> + Rotate<LV>,
     II: Zero + Add<II, II> + InertiaTensor<N, LV, AV, M>>
RayCast<N, LV> for Compound<N, LV, M, II> {
    fn toi_with_ray(&self, ray: &Ray<LV>) -> Option<N> {
        toi_with_ray_on_concave_geom(self, ray)
    }

    fn toi_and_normal_with_ray(&self, ray: &Ray<LV>) -> Option<(N, LV)> {
        toi_and_normal_with_ray_on_concave_geom(self, ray)
    }
}

impl<N:  Clone + Zero + Num + Primitive + Orderable + Cast<f32> + Algebraic,
     LV: Clone + Zero + AlgebraicVecExt<N>,
     AV,
     M:  Clone + Mul<M, M> + Translation<LV> + AbsoluteRotate<LV> + Transform<LV> + Rotate<LV>,
     II: Zero + Add<II, II> + InertiaTensor<N, LV, AV, M>>
RayCastWithTransform<N, LV, M> for Compound<N, LV, M, II> { }

impl<N:  Num + Bounded + Orderable + Primitive + Algebraic + Cast<f32>,
     V:  AlgebraicVecExt<N> + Clone,
     M:  Transform<V> + Rotate<V> + Clone + Mul<M, M> + Translation<V> + AbsoluteRotate<V> + One,
     II: Zero + Add<II, II>,
     E:  Geom<N, V, M, II> + MeshElement<N, V>>
RayCast<N, V> for Mesh<N, V, M, II, E> {
    fn toi_with_ray(&self, ray: &Ray<V>) -> Option<N> {
        // FIXME: using this will do a dynamic dispatch for ray-mesh_element intersection test.
        toi_with_ray_on_concave_geom(self, ray)
    }

    fn toi_and_normal_with_ray(&self, ray: &Ray<V>) -> Option<(N, V)> {
        // FIXME: using this will do a dynamic dispatch for ray-mesh_element intersection test.
        toi_and_normal_with_ray_on_concave_geom(self, ray)
    }
}

impl<N:  Num + Bounded + Orderable + Primitive + Algebraic + Cast<f32>,
     V:  AlgebraicVecExt<N> + Clone,
     M:  Transform<V> + Rotate<V> + Clone + Mul<M, M> + Translation<V> + AbsoluteRotate<V> + One,
     II: Zero + Add<II, II>,
     E:  Geom<N, V, M, II> + MeshElement<N, V>>
RayCastWithTransform<N, V, M> for Mesh<N, V, M, II, E> { }
