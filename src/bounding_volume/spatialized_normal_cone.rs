use crate::bounding_volume::{BoundingVolume, CircularCone, AABB};
use crate::math::Point;
use na::Real;

/// The combination of an AABB with a circular cone to bound both the space occupied by an geometry and its normals.
#[derive(Clone, Debug)]
pub struct SpatializedNormalCone<N: Real> {
    /// An AABB bounding the space occupied by a geometry.
    pub aabb: AABB<N>,
    /// A circular cone bounding the normals of a geometry.
    pub normals: CircularCone<N>,
}

impl<N: Real> BoundingVolume<N> for SpatializedNormalCone<N> {
    fn center(&self) -> Point<N> {
        self.aabb.center()
    }

    fn intersects(&self, other: &Self) -> bool {
        self.aabb.intersects(&other.aabb) && self.normals.double_cones_intersect(&other.normals)
    }

    fn contains(&self, other: &Self) -> bool {
        self.aabb.contains(&other.aabb) && self.normals.contains(&other.normals)
    }

    fn merge(&mut self, other: &Self) {
        self.aabb.merge(&other.aabb);
        self.normals.merge(&other.normals);
    }

    fn merged(&self, other: &Self) -> Self {
        SpatializedNormalCone {
            aabb: self.aabb.merged(&other.aabb),
            normals: self.normals.merged(&other.normals),
        }
    }

    fn loosen(&mut self, margin: N) {
        self.aabb.loosen(margin)
    }

    fn loosened(&self, margin: N) -> Self {
        SpatializedNormalCone {
            aabb: self.aabb.loosened(margin),
            normals: self.normals,
        }
    }

    fn tighten(&mut self, margin: N) {
        self.aabb.tighten(margin)
    }

    fn tightened(&self, margin: N) -> Self {
        SpatializedNormalCone {
            aabb: self.aabb.tightened(margin),
            normals: self.normals,
        }
    }
}
