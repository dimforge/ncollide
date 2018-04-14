use num::{Bounded, Zero};
use na;
use shape::FeatureId;
use query::{PointProjection, PointQuery};
use bounding_volume::AABB;
use math::{Isometry, Point};

impl<N: Real> AABB<N> {
    fn local_point_projection<M>(&self, m: &Isometry<N>, pt: &P, solid: bool) -> (bool, P, Vector<N>)
    where
        M: Isometry<P>,
    {
        let ls_pt = m.inverse_transform_point(pt);
        let mins_pt = *self.mins() - ls_pt;
        let pt_maxs = ls_pt - *self.maxs();
        let shift = na::sup(&na::zero(), &mins_pt) - na::sup(&na::zero(), &pt_maxs);

        let inside = shift == na::zero();

        if !inside {
            (false, ls_pt + shift, shift)
        } else if solid {
            (true, ls_pt, shift)
        } else {
            let _max: N = Bounded::max_value();
            let mut best = -_max;
            let mut best_id = 0isize;

            for i in 0..na::dimension::<Vector<N>>() {
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

            let mut shift: Vector<N> = na::zero();

            if best_id < 0 {
                shift[(-best_id) as usize] = best;
            } else {
                shift[best_id as usize] = -best;
            }

            (inside, ls_pt + shift, shift)
        }
    }
}

impl<N: Real> PointQuery<P, M> for AABB<N> {
    #[inline]
    fn project_point(&self, m: &Isometry<N>, pt: &P, solid: bool) -> PointProjection<P> {
        let (inside, ls_pt, _) = self.local_point_projection(m, pt, solid);
        PointProjection::new(inside, m.transform_point(&ls_pt))
    }

    #[inline]
    fn project_point_with_feature(&self, m: &Isometry<N>, pt: &P) -> (PointProjection<P>, FeatureId) {
        let (inside, ls_pt, shift) = self.local_point_projection(m, pt, false);
        let proj = PointProjection::new(inside, m.transform_point(&ls_pt));
        let dim = na::dimension::<Vector<N>>();
        let mut nzero_shifts = 0;
        let mut last_zero_shift = 0;
        let mut last_not_zero_shift = 0;

        for i in 0..dim {
            if shift[i].is_zero() {
                nzero_shifts += 1;
                last_zero_shift = i;
            } else {
                last_not_zero_shift = i;
            }
        }

        if nzero_shifts > 2 {
            // On a 3D face.
            if ls_pt[last_not_zero_shift] < na::zero() {
                (proj, FeatureId::Face(last_not_zero_shift + dim))
            } else {
                (proj, FeatureId::Face(last_not_zero_shift))
            }
        } else if dim == 2 && nzero_shifts == 1 {
            // On a 2D face.
            if ls_pt[last_not_zero_shift] < na::zero() {
                (proj, FeatureId::Face(last_not_zero_shift + dim))
            } else {
                (proj, FeatureId::Face(last_not_zero_shift))
            }
        } else {
            // On a vertex or edge.
            let mut id = 0;
            for i in 0..dim {
                if ls_pt[i] < na::zero() {
                    id |= 1 << i;
                }
            }
            if nzero_shifts == 0 {
                (proj, FeatureId::Vertex(id))
            } else {
                (proj, FeatureId::Edge((id << 2) | last_zero_shift))
            }
        }
    }

    #[inline]
    fn distance_to_point(&self, m: &Isometry<N>, pt: &P, solid: bool) -> N {
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
    fn contains_point(&self, m: &Isometry<N>, pt: &P) -> bool {
        let ls_pt = m.inverse_transform_point(pt).coords;

        for i in 0..na::dimension::<Vector<N>>() {
            if ls_pt[i] < self.mins()[i] || ls_pt[i] > self.maxs()[i] {
                return false;
            }
        }

        true
    }
}
