//! Support mapping based Cuboid shape.

use num::Zero;

use na::{self, Real, Unit};
use shape::SupportMap;
use math::{Isometry, Point, Vector};

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
        for i in 0..na::dimension::<V>() {
            assert!(half_extents[i] >= V::Real::zero());
        }

        Cuboid {
            half_extents: half_extents,
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

        for i in 0usize..na::dimension::<P::Vector>() {
            if local_dir[i] < P::Real::zero() {
                res[i] = -res[i];
            }
        }

        m.transform_point(&P::from_coordinates(res))
    }

    #[inline]
    fn support_area_toward(&self, m: &M, dir: &Unit<P::Vector>, angle: P::Real, out: &mut Vec<P>) {
        let local_dir = m.inverse_rotate_vector(dir);
        let cang = angle.cos();
        let dim = na::dimension::<P::Vector>();
        let mut half_extents = *self.half_extents();
        let mut support_point = half_extents;

        match dim {
            2 => {
                for i1 in 0 .. 2 {
                    let sign = local_dir[i1].signum();
                    if sign * local_dir[i1] >= cang {
                        let i2 = (i1 + 1) % 2;

                        half_extents[i1] *= sign;
                        half_extents[i2] *= -sign;
                        let p1 = P::from_coordinates(half_extents);

                        half_extents[i2] = -half_extents[i2];
                        let p2 = P::from_coordinates(half_extents);

                        out.push(m.transform_point(&p1));
                        out.push(m.transform_point(&p2));
                        return;
                    }
                    else {
                        support_point[i1] *= sign;
                    }
                }

                // We are not on a face, return the support vertex.
                out.push(m.transform_point(&P::from_coordinates(support_point)));
            },
            3 => {
                unimplemented!()
            },
            _ => out.push(self.support_point_toward(m, dir))
        }
         
    }

}
