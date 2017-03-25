use num::{Bounded, Zero};
use na;
use query::{PointQuery, PointProjection};
use bounding_volume::AABB;
use math::{Point, Isometry};

impl<P: Point, M: Isometry<P>> PointQuery<P, M> for AABB<P> {
    #[inline]
    fn project_point(&self, m: &M, pt: &P, solid: bool) -> PointProjection<P> {
        let ls_pt   = m.inverse_transform_point(pt);
        let mins_pt = *self.mins() - ls_pt;
        let pt_maxs = ls_pt - *self.maxs();
        let shift   = na::sup(&na::zero(), &mins_pt) - na::sup(&na::zero(), &pt_maxs);

        let inside = shift == na::zero();

        if !inside || solid {
            PointProjection::new(inside, *pt + m.rotate_vector(&shift), ())
        }
        else {
            let _max: P::Real = Bounded::max_value();
            let mut best    = -_max;
            let mut best_id = 0isize;

            for i in 0 .. na::dimension::<P::Vector>() {
                let mins_pt_i = mins_pt[i];
                let pt_maxs_i = pt_maxs[i];

                if mins_pt_i < pt_maxs_i {
                    if pt_maxs[i] > best {
                        best_id = i as isize;
                        best    = pt_maxs_i
                    }
                }
                else if mins_pt_i > best {
                    best_id = -(i as isize);
                    best    = mins_pt_i
                }
            }

            let mut shift: P::Vector = na::zero();

            if best_id < 0 {
                shift[(-best_id) as usize] = best;
            }
            else {
                shift[best_id as usize] = -best;
            }

            PointProjection::new(inside, *pt + m.rotate_vector(&shift), ())
        }
    }

    #[inline]
    fn distance_to_point(&self, m: &M, pt: &P, solid: bool) -> P::Real {
        let ls_pt   = m.inverse_transform_point(pt);
        let mins_pt = *self.mins() - ls_pt;
        let pt_maxs = ls_pt - *self.maxs();
        let shift = na::sup(&na::zero(), &na::sup(&mins_pt, &pt_maxs));

        if solid || !shift.is_zero() {
            na::norm(&shift)
        }
        else {
            // FIXME: optimize that.
            -na::distance(pt, &self.project_point(m, pt, solid).point)
        }
    }

    #[inline]
    fn contains_point(&self, m: &M, pt: &P) -> bool {
        let ls_pt = m.inverse_transform_point(pt).coordinates();

        for i in 0 .. na::dimension::<P::Vector>() {
            if ls_pt[i] < self.mins()[i] || ls_pt[i] > self.maxs()[i] {
                return false
            }
        }

        true
    }
}
