//! Support mapping based Cuboid shape.

use num::Zero;

use na;
use shape::SupportMap;
use math::{Point, Vector, Isometry};

/// Shape of a box.
#[derive(PartialEq, Debug, Clone, RustcEncodable, RustcDecodable)]
pub struct Cuboid<V> {
    half_extents: V,
}

impl<V: Vector> Cuboid<V> {
    /// Creates a new box from its half-extents. Half-extents are the box half-width along each
    /// axis. Each half-extent must be greater than 0.04.
    #[inline]
    pub fn new(half_extents: V) -> Cuboid<V> {
        for i in 0 .. na::dimension::<V>() {
            assert!(half_extents[i] >= V::Real::zero());
        }

        Cuboid {
            half_extents: half_extents
        }
    }
}

impl<V> Cuboid<V> {
    /// The half-extents of this box. Half-extents are the box half-width along each axis.
    #[inline]
    pub fn half_extents(&self) -> &V {
        &self.half_extents
    }
}

impl<P: Point, M: Isometry<P>> SupportMap<P, M> for Cuboid<P::Vector> {
    #[inline]
    fn support_point(&self, m: &M, dir: &P::Vector) -> P {
        let local_dir = m.inverse_rotate_vector(dir);

        let mut res = *self.half_extents();

        for i in 0usize .. na::dimension::<P::Vector>() {
            if local_dir[i] < P::Real::zero() {
                res[i] = -res[i];
            }
        }

        m.transform_point(&P::from_coordinates(res))
    }
}
