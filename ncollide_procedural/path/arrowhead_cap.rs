use alga::general::Real;
use na::{Point3, Vector3, Isometry3};
use na;
use path::PolylineCompatibleCap;
use polyline::Polyline;
use utils;

/// A cap that looks like an arrow.
pub struct ArrowheadCap<N> {
    radius_scale:       N,
    front_dist_to_head: N,
    back_dist_to_head:  N
}

impl<N: Real> ArrowheadCap<N> {
    /// Creates a cap that looks like an arrow.
    ///
    /// # Arguments:
    /// * `radius_scale` - scale factor of the cap base.
    /// * `front_dist_to_head` - distance from the path endpoint and the arrow tip.
    /// * `back_dist_to_head` - distance from the path endpoint and the cap base.
    pub fn new(radius_scale: N, front_dist_to_head: N, back_dist_to_head: N) -> ArrowheadCap<N> {
        ArrowheadCap {
            radius_scale:       radius_scale,
            front_dist_to_head: front_dist_to_head,
            back_dist_to_head:  back_dist_to_head
        }
    }

    fn do_gen_cap(&self,
                  attach_id:       u32,
                  pattern:         &Polyline<Point3<N>>,
                  pt:              &Point3<N>,
                  dir:             &Vector3<N>,
                  closed:          bool,
                  negative_shifts: bool,
                  coords:          &mut Vec<Point3<N>>,
                  indices:         &mut Vec<Point3<u32>>) {
        let front_dist_to_head = if negative_shifts { -self.front_dist_to_head } else { self.front_dist_to_head };
        let back_dist_to_head  = if negative_shifts { -self.back_dist_to_head } else { self.back_dist_to_head };
        let pointy_thing  = *pt + *dir * front_dist_to_head;
        let start_id      = coords.len() as u32;
        let npts          = pattern.coords().len() as u32;
        let mut attach_id = attach_id;

        if !(self.radius_scale == na::convert(1.0)) || !back_dist_to_head.is_zero() {
            let mut new_pattern = pattern.clone();
            new_pattern.scale_by_scalar(&self.radius_scale);

            // NOTE: this is done exactly the same on the PolylinePattern::stroke method.
            // Refactor?
            let transform;
            let back_shift = *dir * back_dist_to_head;

            if dir.x.is_zero() && dir.z.is_zero() { // FIXME: this might not be enough to avoid singularities.
                transform = Isometry3::new_observer_frame(&(*pt - back_shift), &(*pt + *dir), &Vector3::x());
            }

            else {
                transform = Isometry3::new_observer_frame(&(*pt - back_shift), &(*pt + *dir), &Vector3::y());
            }

            new_pattern.transform_by(&transform);

            coords.extend(new_pattern.coords().into_iter());

            if closed {
                utils::push_ring_indices(attach_id, start_id, npts, indices)
            }
            else {
                utils::push_open_ring_indices(attach_id, start_id, npts, indices)
            }

            attach_id = start_id;
        }

        if closed {
            utils::push_degenerate_top_ring_indices(attach_id, coords.len() as u32 , npts, indices);
        }
        else {
            utils::push_degenerate_open_top_ring_indices(attach_id, coords.len() as u32 , npts, indices);
        }

        coords.push(pointy_thing);
    }
}

impl<N: Real> PolylineCompatibleCap<N> for ArrowheadCap<N> {
    fn gen_end_cap(&self,
                   attach_id: u32,
                   pattern:   &Polyline<Point3<N>>,
                   pt:        &Point3<N>,
                   dir:       &Vector3<N>,
                   closed:    bool,
                   coords:    &mut Vec<Point3<N>>,
                   indices:   &mut Vec<Point3<u32>>) {
        let start_indices_id = indices.len();

        self.do_gen_cap(attach_id, pattern, pt, dir, closed, false, coords, indices);
        utils::reverse_clockwising(&mut indices[start_indices_id ..])
    }

    fn gen_start_cap(&self,
                     attach_id: u32,
                     pattern:   &Polyline<Point3<N>>,
                     pt:        &Point3<N>,
                     dir:       &Vector3<N>,
                     closed:    bool,
                     coords:    &mut Vec<Point3<N>>,
                     indices:   &mut Vec<Point3<u32>>) {
        self.do_gen_cap(attach_id, pattern, pt, dir, closed, true, coords, indices)
    }
}
