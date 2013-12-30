use std::num::Zero;
use nalgebra::na::{AlgebraicVecExt, Rotate, Transform, Cast, Translation, AbsoluteRotate};
use nalgebra::na;
use volumetric::InertiaTensor;
use ray::{Ray, RayCast, RayCastWithTransform};
use geom::{ConcaveGeom, Compound};

impl<N:  Clone + Zero + Num + Primitive + Orderable + Cast<f32> + Algebraic,
     LV: Clone + Zero + AlgebraicVecExt<N>,
     AV,
     M:  Clone + Mul<M, M> + Translation<LV> + AbsoluteRotate<LV> + Transform<LV> + Rotate<LV>,
     II: Zero + Add<II, II> + InertiaTensor<N, LV, AV, M>>
RayCast<N, LV> for Compound<N, LV, M, II> {
    fn toi_with_ray(&self, ray: &Ray<LV>) -> Option<N> {
        let mut interferences: ~[uint] = ~[];

        self.approx_interferences_with_ray(ray, &mut interferences);

        // compute the minimum toi
        let mut toi: N = Bounded::max_value();

        for i in interferences.iter() {
            self.map_part_at(*i, |objm, obj|
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

    fn toi_and_normal_with_ray(&self, ray: &Ray<LV>) -> Option<(N, LV)> {
        let mut interferences: ~[uint] = ~[];

        self.approx_interferences_with_ray(ray, &mut interferences);

        // compute the minimum toi
        let mut toi: (N, LV) = (Bounded::max_value(), na::zero());

        for i in interferences.iter() {
            self.map_part_at(*i, |objm, obj|
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

    // XXX: we have to implement toi_and_normal_and_uv_with_ray! Otherwise, no uv will be computed
    // for any of the sub-shapes.
}

impl<N:  Clone + Zero + Num + Primitive + Orderable + Cast<f32> + Algebraic,
     LV: Clone + Zero + AlgebraicVecExt<N>,
     AV,
     M:  Clone + Mul<M, M> + Translation<LV> + AbsoluteRotate<LV> + Transform<LV> + Rotate<LV>,
     II: Zero + Add<II, II> + InertiaTensor<N, LV, AV, M>>
RayCastWithTransform<N, LV, M> for Compound<N, LV, M, II> { }
