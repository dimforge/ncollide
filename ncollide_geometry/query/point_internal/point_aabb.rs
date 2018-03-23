use num::{Bounded, Zero};
use na;
use shape::FeatureId;
use query::{PointProjection, PointQuery};
use bounding_volume::AABB;
use math::{Isometry, Point};

impl<P: Point> AABB<P> {
    fn point_projection_local_shift<M>(&self, m: &M, pt: &P, solid: bool) -> (bool, P::Vector)
    where
        M: Isometry<P>,
    {
        let ls_pt = m.inverse_transform_point(pt);
        let mins_pt = *self.mins() - ls_pt;
        let pt_maxs = ls_pt - *self.maxs();
        let shift = na::sup(&na::zero(), &mins_pt) - na::sup(&na::zero(), &pt_maxs);

        let inside = shift == na::zero();

        if !inside || solid {
            (inside, shift)
        } else {
            let _max: P::Real = Bounded::max_value();
            let mut best = -_max;
            let mut best_id = 0isize;

            for i in 0..na::dimension::<P::Vector>() {
                let mins_pt_i = mins_pt[i];
                let pt_maxs_i = pt_maxs[i];

                if mins_pt_i < pt_maxs_i {
                    if pt_maxs[i] > best {
                        best_id = i as isize;
                        best = pt_maxs_i
                    }
                } else if mins_pt_i > best {
                    best_id = -(i as isize);
                    best = mins_pt_i
                }
            }

            let mut shift: P::Vector = na::zero();

            if best_id < 0 {
                shift[(-best_id) as usize] = best;
            } else {
                shift[best_id as usize] = -best;
            }

            (inside, shift)
        }
    }
}

impl<P: Point, M: Isometry<P>> PointQuery<P, M> for AABB<P> {
    #[inline]
    fn project_point(&self, m: &M, pt: &P, solid: bool) -> PointProjection<P> {
        let (inside, shift) = self.point_projection_local_shift(m, pt, solid);
        PointProjection::new(inside, *pt + m.rotate_vector(&shift))
    }

    #[inline]
    fn project_point_with_feature(&self, m: &M, pt: &P) -> (PointProjection<P>, FeatureId) {
        let (inside, shift) = self.point_projection_local_shift(m, pt, false);
        let proj = PointProjection::new(inside, *pt + m.rotate_vector(&shift));
        let dim = na::dimension::<P::Vector>();
        let mut nzeros = 0;
        let mut last_zero = 0;
        let mut last_nonzero = 0;
        for i in 0..dim {
            if shift[i] == na::zero() {
                nzeros += 1;
                last_zero = i;
            } else {
                last_nonzero = i;
            }
        }

        if nzeros < 2 {
            // On a vertex or edge.
            let mut id = 0;
            for i in 0..dim {
                if shift[i] < na::zero() {
                    id |= 1 << i;
                }
            }
            if nzeros == 0 {
                (proj, FeatureId::vertex(0, id))
            } else {
                (proj, FeatureId::edge(0, (id << dim) | last_zero))
            }
        } else {
            // On a face.
            if shift[last_nonzero] < na::zero() {
                (proj, FeatureId::face(0, last_nonzero + 3))
            } else {
                (proj, FeatureId::face(0, last_nonzero))
            }
        }
    }

    #[inline]
    fn distance_to_point(&self, m: &M, pt: &P, solid: bool) -> P::Real {
        let ls_pt = m.inverse_transform_point(pt);
        let mins_pt = *self.mins() - ls_pt;
        let pt_maxs = ls_pt - *self.maxs();
        let shift = na::sup(&na::zero(), &na::sup(&mins_pt, &pt_maxs));

        if solid || !shift.is_zero() {
            na::norm(&shift)
        } else {
            // FIXME: optimize that.
            -na::distance(pt, &self.project_point(m, pt, solid).point)
        }
    }

    #[inline]
    fn contains_point(&self, m: &M, pt: &P) -> bool {
        let ls_pt = m.inverse_transform_point(pt).coordinates();

        for i in 0..na::dimension::<P::Vector>() {
            if ls_pt[i] < self.mins()[i] || ls_pt[i] > self.maxs()[i] {
                return false;
            }
        }

        true
    }
}
