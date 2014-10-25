use std::num::Bounded;
use na::{Translate, AbsoluteRotate, Rotate, Transform};
use na;
use ray::{Ray, LocalRayCast, RayCast, RayIntersection};
use geom::{ConcaveGeom, Compound};
use math::{Scalar, Point, Vect};


// XXX: if solid == false, this might return internal intersection.
impl<N, P, V, M, I> LocalRayCast<N, P, V> for Compound<N, P, V, M, I>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P>,
          M: Send + AbsoluteRotate<V> + Transform<P> + Rotate<V> + Mul<M, M> + Clone,
          I: Send + Clone {
    fn toi_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<N> {
        // FIXME: optimize that and avoid the allocation using the dedicated ray casting function
        // from the BVT.
        let mut interferences: Vec<uint> = Vec::new();

        self.approx_interferences_with_ray(ray, &mut interferences);

        // compute the minimum toi
        let mut toi: N = Bounded::max_value();

        for i in interferences.iter() {
            self.map_part_at(*i, |objm, obj|
                          match obj.toi_with_transform_and_ray(objm, ray, solid) {
                              None        => { },
                              Some(ref t) => toi = toi.min(*t)
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

    fn toi_and_normal_with_ray(&self, ray: &Ray<P, V>, solid: bool) -> Option<RayIntersection<N, V>> {
        let mut interferences: Vec<uint> = Vec::new();

        self.approx_interferences_with_ray(ray, &mut interferences);

        // compute the minimum toi
        let mut best = RayIntersection::new(Bounded::max_value(), na::zero());

        for i in interferences.iter() {
            self.map_part_at(*i, |objm, obj|
                          match obj.toi_and_normal_with_transform_and_ray(objm, ray, solid) {
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

impl<N, P, V, M, I> RayCast<N, P, V, M> for Compound<N, P, V, M, I>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P>,
          M: Send + AbsoluteRotate<V> + Transform<P> + Rotate<V> + Mul<M, M> + Clone,
          I: Send + Clone {
}
