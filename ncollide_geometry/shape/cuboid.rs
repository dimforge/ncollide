//! Support mapping based Cuboid shape.

use na::{self, Transform, Rotate};
use shape::SupportMap;
use math::{Point, Vector};

/// Shape of a box.
#[derive(PartialEq, Debug, Clone, RustcEncodable, RustcDecodable)]
pub struct Cuboid<V> {
    half_extents: V,
}

impl<V> Copy for Cuboid<V> where V: Copy {}

impl<V: Vector> Cuboid<V> {
    /// Creates a new box from its half-extents. Half-extents are the box half-width along each
    /// axis. Each half-extent must be greater than 0.04.
    #[inline]
    pub fn new(half_extents: V) -> Cuboid<V> {
        assert!(half_extents.iter().all(|e| *e >= na::zero()));

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

impl<P, M> SupportMap<P, M> for Cuboid<P::Vect>
    where P: Point,
          M: Rotate<P::Vect> + Transform<P> {
    #[inline]
    fn support_point(&self, m: &M, dir: &P::Vect) -> P {
        let local_dir = m.inverse_rotate(dir);

        let mut pres: P = na::origin();

        let he = self.half_extents();
        for i in 0usize .. na::dimension::<P>() {
            if local_dir[i] < na::zero() {
                pres[i] = -he[i];
            }
            else {
                pres[i] = he[i];
            }
        }

        m.transform(&pres)
    }
}
