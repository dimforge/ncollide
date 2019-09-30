use na::RealField;
use crate::sampling::SurfaceSampling;
use crate::shape::{Cuboid, FeatureId};
use crate::math::Point;

#[cfg(feature = "dim2")]
impl<N: RealField> SurfaceSampling<N> for Cuboid<N> {
    fn sample_feature(&self, feature: FeatureId, spacing: N, out: &mut Vec<Point<N>>) {
        unimplemented!()
    }

    fn sample_surface(&self, spacing: N, out: &mut Vec<Point<N>>) {
        let half_extents = self.half_extents();
        let mut curr = Point::from(-half_extents);

        while curr.x < half_extents.x {
            out.push(curr);
            curr.x += spacing;
        }

        curr.x = half_extents.x;

        while curr.y < half_extents.y {
            out.push(curr);
            curr.y += spacing;
        }

        curr.y = half_extents.y;

        while curr.x > -half_extents.x {
            out.push(curr);
            curr.x -= spacing;
        }

        curr.x = -half_extents.x;

        while curr.y > -half_extents.y {
            out.push(curr);
            curr.y -= spacing;
        }
    }
}

#[cfg(feature = "dim3")]
impl<N: RealField> SurfaceSampling<N> for Cuboid<N> {
    fn sample_feature(&self, feature: FeatureId, spacing: N, out: &mut Vec<Point<N>>) {
        unimplemented!()
    }

    fn sample_surface(&self, spacing: N, out: &mut Vec<Point<N>>) {
        unimplemented!()
    }
}

