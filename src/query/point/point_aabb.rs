use crate::bounding_volume::AABB;
use crate::math::{Isometry, Point, Vector, DIM};
use crate::num::{Bounded, Zero};
use crate::query::{PointProjection, PointQuery};
use crate::shape::FeatureId;
use na::{self, RealField};

impl<N: RealField> AABB<N> {
    fn local_point_projection(
        &self,
        m: &Isometry<N>,
        pt: &Point<N>,
        solid: bool,
    ) -> (bool, Point<N>, Vector<N>) {
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
            let mut is_mins = false;
            let mut best_id = 0;

            for i in 0..DIM {
                let mins_pt_i = mins_pt[i];
                let pt_maxs_i = pt_maxs[i];

                if mins_pt_i < pt_maxs_i {
                    if pt_maxs[i] > best {
                        best_id = i;
                        is_mins = false;
                        best = pt_maxs_i
                    }
                } else if mins_pt_i > best {
                    best_id = i;
                    is_mins = true;
                    best = mins_pt_i
                }
            }

            let mut shift: Vector<N> = na::zero();

            if is_mins {
                shift[best_id] = best;
            } else {
                shift[best_id] = -best;
            }

            (inside, ls_pt + shift, shift)
        }
    }
}

impl<N: RealField> PointQuery<N> for AABB<N> {
    #[inline]
    fn project_point(&self, m: &Isometry<N>, pt: &Point<N>, solid: bool) -> PointProjection<N> {
        let (inside, ls_pt, _) = self.local_point_projection(m, pt, solid);
        PointProjection::new(inside, m * ls_pt)
    }

    #[allow(unused_assignments)] // For last_zero_shift which is used only in 3D.
    #[allow(unused_variables)] // For last_zero_shift which is used only in 3D.
    #[inline]
    fn project_point_with_feature(
        &self,
        m: &Isometry<N>,
        pt: &Point<N>,
    ) -> (PointProjection<N>, FeatureId) {
        let (inside, ls_pt, shift) = self.local_point_projection(m, pt, false);
        let proj = PointProjection::new(inside, m * ls_pt);
        let mut nzero_shifts = 0;
        let mut last_zero_shift = 0;
        let mut last_not_zero_shift = 0;

        for i in 0..DIM {
            if shift[i].is_zero() {
                nzero_shifts += 1;
                last_zero_shift = i;
            } else {
                last_not_zero_shift = i;
            }
        }

        if nzero_shifts == DIM {
            for i in 0..DIM {
                if ls_pt[i] > self.maxs()[i] - N::default_epsilon() {
                    return (proj, FeatureId::Face(i));
                }
                if ls_pt[i] <= self.mins()[i] + N::default_epsilon() {
                    return (proj, FeatureId::Face(i + DIM));
                }
            }

            (proj, FeatureId::Unknown)
        } else if nzero_shifts == DIM - 1 {
            // On a 3D face.
            if ls_pt[last_not_zero_shift] < self.center()[last_not_zero_shift] {
                (proj, FeatureId::Face(last_not_zero_shift + DIM))
            } else {
                (proj, FeatureId::Face(last_not_zero_shift))
            }
        } else {
            // On a vertex or edge.
            let mut id = 0;
            let center = self.center();

            for i in 0..DIM {
                if ls_pt[i] < center[i] {
                    id |= 1 << i;
                }
            }

            #[cfg(feature = "dim3")]
            {
                if nzero_shifts == 0 {
                    (proj, FeatureId::Vertex(id))
                } else {
                    (proj, FeatureId::Edge((id << 2) | last_zero_shift))
                }
            }

            #[cfg(feature = "dim2")]
            {
                (proj, FeatureId::Vertex(id))
            }
        }
    }

    #[inline]
    fn distance_to_point(&self, m: &Isometry<N>, pt: &Point<N>, solid: bool) -> N {
        let ls_pt = m.inverse_transform_point(pt);
        let mins_pt = *self.mins() - ls_pt;
        let pt_maxs = ls_pt - *self.maxs();
        let shift = na::sup(&na::zero(), &na::sup(&mins_pt, &pt_maxs));

        if solid || !shift.is_zero() {
            shift.norm()
        } else {
            // FIXME: optimize that.
            -na::distance(pt, &self.project_point(m, pt, solid).point)
        }
    }

    #[inline]
    fn contains_point(&self, m: &Isometry<N>, pt: &Point<N>) -> bool {
        let ls_pt = m.inverse_transform_point(pt);
        self.contains_local_point(&ls_pt)
    }
}
