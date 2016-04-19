use num::Zero;
use na::{Transform, Rotate, Bounded};
use na;
use point::{PointQuery, PointProjection};
use entities::bounding_volume::AABB;
use math::{Point, Vector};

impl<P, M> PointQuery<P, M> for AABB<P>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
    #[inline]
    fn project_point(&self, m: &M, pt: &P, solid: bool) -> PointProjection<P> {
        let ls_pt   = m.inverse_transform(pt);
        let mins_pt = *self.mins() - ls_pt;
        let pt_maxs = ls_pt - *self.maxs();
        let shift   = na::sup(&na::zero(), &mins_pt) - na::sup(&na::zero(), &pt_maxs);

        let inside = shift.is_zero();

        if !inside || solid {
            PointProjection::new(inside, *pt + m.rotate(&shift))
        }
        else {
            let _max: <P::Vect as Vector>::Scalar = Bounded::max_value();
            let mut best    = -_max;
            let mut best_id = 0isize;

            for i in 0 .. na::dimension::<P::Vect>() {
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

            let mut shift: P::Vect = na::zero();

            if best_id < 0 {
                shift[(-best_id) as usize] = best;
            }
            else {
                shift[best_id as usize] = -best;
            }

            PointProjection::new(inside, *pt + m.rotate(&shift))
        }
    }

    #[inline]
    fn distance_to_point(&self, m: &M, pt: &P, solid: bool) -> <P::Vect as Vector>::Scalar {
        let ls_pt   = m.inverse_transform(pt);
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
        let ls_pt = m.inverse_transform(pt);

        for i in 0 .. na::dimension::<P>() {
            if ls_pt[i] < self.mins()[i] || ls_pt[i] > self.maxs()[i] {
                return false
            }
        }

        true
    }
}
