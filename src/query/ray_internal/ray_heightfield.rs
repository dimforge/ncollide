use bounding_volume::AABB;
use math::Isometry;
use na::{Point2, Real, Vector3};
use partitioning::{BestFirstBVVisitStatus, BestFirstDataVisitStatus, BestFirstVisitor};
use query::{closest_points_internal, Ray, RayCast, RayIntersection};
use shape::{CompositeShape, HeightField, FeatureId};

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
        let (clip_ray_a, clip_ray_b) = (ls_ray.point_at(min_t), ls_ray.point_at(max_t));
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
                return Some(RayIntersection::new(s, seg.normal().unwrap().unwrap(), FeatureId::Face(curr)))
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
                let (s, t) = closest_points_internal::line_against_line_parameters(&ray.origin, &ray.dir, seg.a(), &seg.scaled_direction());
                if t >= N::zero() && t <= N::one() {
                    return Some(RayIntersection::new(s, seg.normal().unwrap().unwrap(), FeatureId::Face(curr)))
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
        _: bool,
    ) -> Option<RayIntersection<N>>
    {
        None
    }
}
