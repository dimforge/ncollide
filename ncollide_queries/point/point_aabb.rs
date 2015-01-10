use na::{Transform, Bounded};
use na;
use point::{LocalPointQuery, PointQuery};
use entities::bounding_volume::AABB;
use math::{Scalar, Point, Vect};

#[old_impl_check]
impl<N, P, V> LocalPointQuery<N, P> for AABB<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    #[inline]
    fn project_point(&self, pt: &P, solid: bool) -> P {
        let mins_pt = *self.mins() - *pt;
        let pt_maxs = *pt - *self.maxs();
        let shift   = na::sup(&na::zero(), &mins_pt) - na::sup(&na::zero(), &pt_maxs);

        if !shift.is_zero() || solid {
            *pt + shift
        }
        else {
            let _max: N     = Bounded::max_value();
            let mut best    = -_max;
            let mut best_id = 0i;

            for i in range(0, na::dim::<V>()) {
                let mins_pt_i = mins_pt[i];
                let pt_maxs_i = pt_maxs[i];

                if mins_pt_i < pt_maxs_i {
                    if pt_maxs[i] > best {
                        best_id = i as int;
                        best = pt_maxs_i
                    }
                }
                else if mins_pt_i > best {
                    best_id = -(i as int);
                    best = mins_pt_i
                }
            }

            let mut shift: V = na::zero();

            if best_id < 0 {
                shift[(-best_id) as uint] = best;
            }
            else {
                shift[best_id as uint] = -best;
            }

            *pt + shift
        }
    }

    #[inline]
    fn distance_to_point(&self, pt: &P) -> N {
        let mins_pt = *self.mins() - *pt;
        let pt_maxs = *pt - *self.maxs();

        na::norm(&na::sup(&na::zero(), &na::sup(&mins_pt, &pt_maxs)))
    }

    #[inline]
    fn contains_point(&self, pt: &P) -> bool {
        for i in range(0, na::dim::<P>()) {
            if pt[i] < self.mins()[i] || pt[i] > self.maxs()[i] {
                return false
            }
        }

        true
    }
}

#[old_impl_check]
impl<N, P, V, M> PointQuery<N, P, M> for AABB<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> {
}
