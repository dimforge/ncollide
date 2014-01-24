use std::num::Bounded;
use nalgebra::na;
use ray::{Ray, RayCast, RayIntersection};
use geom::{ConcaveGeom, Compound};
use math::N;

impl RayCast for Compound {
    fn toi_with_ray(&self, ray: &Ray) -> Option<N> {
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

    fn toi_and_normal_with_ray(&self, ray: &Ray) -> Option<RayIntersection> {
        let mut interferences: ~[uint] = ~[];

        self.approx_interferences_with_ray(ray, &mut interferences);

        // compute the minimum toi
        let mut best = RayIntersection::new(Bounded::max_value(), na::zero());

        for i in interferences.iter() {
            self.map_part_at(*i, |objm, obj|
                          match obj.toi_and_normal_with_transform_and_ray(objm, ray) {
                              None        => { },
                              Some(inter) => {
                                  if inter.toi < best.toi {
                                      best = inter
                                  }
                              }
                          }
                         );
        }

        if best.toi == Bounded::max_value() {
            None
        }
        else {
            Some(best)
        }
    }

    // XXX: we have to implement toi_and_normal_and_uv_with_ray! Otherwise, no uv will be computed
    // for any of the sub-shapes.
}
