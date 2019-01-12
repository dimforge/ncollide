use crate::math::Isometry;
use na::Real;
use crate::query::{Ray, RayCast, RayIntersection};
use crate::shape::HeightField;
#[cfg(feature = "dim2")]
use crate::shape::FeatureId;
#[cfg(feature = "dim2")]
use crate::query::closest_points_internal;


#[cfg(feature = "dim2")]
impl<N: Real> RayCast<N> for HeightField<N> {
    #[inline]
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        _: bool,
    ) -> Option<RayIntersection<N>>
    {
        let aabb = self.aabb();
        let ls_ray = ray.inverse_transform_by(m);
        let (min_t, max_t) = aabb.clip_ray_parameters(&ls_ray)?;
        let (clip_ray_a, _clip_ray_b) = (ls_ray.point_at(min_t), ls_ray.point_at(max_t));
        let mut curr = match self.cell_at_point(&clip_ray_a) {
            Some(cell) => cell,
            // None may happen due to slight numerical errors.
            None => if ls_ray.origin.x > N::zero() { self.num_cells() - 1 } else {
                0
            }
        };

        /*
         * Test the segment under the ray.
         */
        if let Some(seg) = self.segment_at(curr) {
            let (s, t) = closest_points_internal::line_against_line_parameters(&ray.origin, &ray.dir, seg.a(), &seg.scaled_direction());
            if s >= N::zero() && t >= N::zero() && t <= N::one() {
                // Cast succeeded on the first element!
                let n = seg.normal().unwrap().unwrap();
                let fid = if n.dot(&ls_ray.dir) > N::zero() {
                    // The ray hit the back face.
                    curr + self.num_cells()
                } else {
                    // The ray hit the front face.
                    curr
                };

                return Some(RayIntersection::new(s, m * n, FeatureId::Face(fid)))
            }
        }

        /*
         * Test other segments in the path of the ray.
         */
        if ls_ray.dir.x == N::zero() {
            return None;
        }

        let right = ls_ray.dir.x > N::zero();
        let cell_width = self.cell_width();
        let start_x = self.start_x();

        while (right && curr < self.num_cells()) || (!right && curr > 0) {
            let curr_param;

            if right {
                curr += 1;
                curr_param = (cell_width * na::convert(curr as f64) + start_x - ls_ray.origin.x) / ls_ray.dir.x;
            } else {
                curr_param = (ls_ray.origin.x - cell_width * na::convert(curr as f64) - start_x) / ls_ray.dir.x;
                curr -= 1;
            }

            if curr_param >= max_t {
                // The part of the ray after max_t is outside of the heightfield AABB.
                return None;
            }

            if let Some(seg) = self.segment_at(curr) {
                // TODO: test the y-coordinates (equivalent to an AABB test) before actually computing the intersection.
                let (s, t) = closest_points_internal::line_against_line_parameters(&ray.origin, &ray.dir, seg.a(), &seg.scaled_direction());
                if t >= N::zero() && t <= N::one() {
                    let n = seg.normal().unwrap().unwrap();
                    let fid = if n.dot(&ls_ray.dir) > N::zero() {
                        // The ray hit the back face.
                        curr + self.num_cells()
                    } else {
                        // The ray hit the front face.
                        curr
                    };
                    return Some(RayIntersection::new(s, m * n, FeatureId::Face(fid)))
                }
            }
        }

        None
    }
}


#[cfg(feature = "dim3")]
impl<N: Real> RayCast<N> for HeightField<N> {
    #[inline]
    fn toi_and_normal_with_ray(
        &self,
        m: &Isometry<N>,
        ray: &Ray<N>,
        solid: bool,
    ) -> Option<RayIntersection<N>>
    {
        let aabb = self.aabb();
        let ls_ray = ray.inverse_transform_by(m);
        let (min_t, max_t) = aabb.clip_ray_parameters(&ls_ray)?;
        // FIXME: take _clip_ray_b into account to stop the loop bellow.
        let (clip_ray_a, _clip_ray_b) = (ls_ray.point_at(min_t), ls_ray.point_at(max_t));
        let mut cell = match self.cell_at_point(&clip_ray_a) {
            Some(cell) => cell,
            // None may happen due to slight numerical errors.
            None => {
                let i = if ls_ray.origin.z > N::zero() {
                    self.nrows() - 1
                } else {
                    0
                };

                let j = if ls_ray.origin.x > N::zero() {
                    self.ncols() - 1
                } else {
                    0
                };

                (i, j)
            }
        };

        loop {
            let tris = self.triangles_at(cell.0, cell.1);
            let inter1 = tris.0.and_then(|tri| tri.toi_and_normal_with_ray(m, ray, solid));
            let inter2 = tris.1.and_then(|tri| tri.toi_and_normal_with_ray(m, ray, solid));

            match (inter1, inter2) {
                (Some(mut inter1), Some(mut inter2)) => {
                    if inter1.toi < inter2.toi {
                        inter1.feature = self.convert_triangle_feature_id(cell.0, cell.1, true, inter1.feature);
                        return Some(inter1);
                    } else {
                        inter2.feature = self.convert_triangle_feature_id(cell.0, cell.1, false, inter2.feature);
                        return Some(inter2);
                    }
                }
                (Some(mut inter), None) => {
                    inter.feature = self.convert_triangle_feature_id(cell.0, cell.1, true, inter.feature);
                    return Some(inter);
                }
                (None, Some(mut inter)) => {
                    inter.feature = self.convert_triangle_feature_id(cell.0, cell.1, false, inter.feature);
                    return Some(inter);
                }
                (None, None) => {}
            }

            /*
             * Find the next cell to cast the ray on.
             */
            let (toi_x, right) = if ls_ray.dir.x > N::zero() {
                let x = self.x_at(cell.1 + 1);
                ((x - ls_ray.origin.x) / ls_ray.dir.x, true)
            } else if ls_ray.dir.x < N::zero() {
                let x = self.x_at(cell.1 + 0);
                ((x - ls_ray.origin.x) / ls_ray.dir.x, false)
            } else {
                (N::max_value(), false)
            };

            let (toi_z, down) = if ls_ray.dir.z > N::zero() {
                let z = self.z_at(cell.0 + 1);
                ((z - ls_ray.origin.z) / ls_ray.dir.z, true)
            } else if ls_ray.dir.z < N::zero() {
                let z = self.z_at(cell.0 + 0);
                ((z - ls_ray.origin.z) / ls_ray.dir.z, false)
            } else {
                (N::max_value(), false)
            };

            if toi_x == N::max_value() && toi_z == N::max_value() {
                break;
            }

            if toi_x >= N::zero() && toi_x < toi_z {
                if right {
                    cell.1 += 1
                } else if cell.1 > 0 {
                    cell.1 -= 1
                } else {
                    break;
                }
            } else if toi_z >= N::zero() {
                if down {
                    cell.0 += 1
                } else if cell.0 > 0 {
                    cell.0 -= 1
                } else {
                    break
                }
            } else {
                break;
            }

            if cell.0 >= self.nrows() || cell.1 >= self.ncols() {
                break;
            }
        }

        None
    }
}
