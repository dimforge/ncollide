use crate::{
    bounding_volume::{HasBoundingVolume, AABB},
    math::{Isometry, Point, DIM},
    shape::Triangle,
};
use na::RealField;

impl<N: RealField + Copy> HasBoundingVolume<N, AABB<N>> for Triangle<N> {
    #[inline]
    fn bounding_volume(&self, m: &Isometry<N>) -> AABB<N> {
        let a = m.transform_point(&self.a).coords;
        let b = m.transform_point(&self.b).coords;
        let c = m.transform_point(&self.c).coords;

        let mut min = Point::origin();
        let mut max = Point::origin();

        for d in 0..DIM {
            min.coords[d] = a[d].min(b[d]).min(c[d]);
            max.coords[d] = a[d].max(b[d]).max(c[d]);
        }

        AABB::new(min, max)
    }

    #[inline]
    fn local_bounding_volume(&self) -> AABB<N> {
        let a = self.a.coords;
        let b = self.b.coords;
        let c = self.c.coords;

        let mut min = Point::origin();
        let mut max = Point::origin();

        for d in 0..DIM {
            min.coords[d] = a[d].min(b[d]).min(c[d]);
            max.coords[d] = a[d].max(b[d]).max(c[d]);
        }

        AABB::new(min, max)
    }
}

#[cfg(test)]
#[cfg(feature = "dim3")]
mod test {
    use crate::{
        bounding_volume::support_map_aabb,
        math::{Isometry, Point, Translation},
        shape::{Shape, Triangle},
    };
    use na::{RealField, UnitQuaternion};

    #[test]
    fn triangle_aabb_matches_support_map_aabb() {
        let t = Triangle::new(
            Point::new(0.3, -0.1, 0.2),
            Point::new(-0.7, 1.0, 0.0),
            Point::new(-0.7, 1.5, 0.0),
        );

        let m = Isometry::from_parts(
            Translation::new(-0.2, 5.0, 0.2),
            UnitQuaternion::from_euler_angles(0.0, f32::frac_pi_2(), 0.0),
        );

        assert_eq!(t.aabb(&m), support_map_aabb(&m, &t));

        // TODO: also test local AABB once support maps have a local AABB
        // function too
    }
}
