//! Support mapping based Cuboid shape.

use num::Float;
use na::{self, Transform, Rotate};
use shape::SupportMap;
use math::{Point, Vector};

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

    fn support_point_set(&self,
                         m:          &M,
                         dir:        &P::Vect,
                         angtol:     <P::Vect as Vector>::Scalar,
                         _:          usize,
                         out_points: &mut Vec<P>)
                         -> usize {
        assert!(na::dimension::<P>() <= 3,
                "Set-valued support point computation not yet implemented for dimensions > 3.");

        let local_dir = na::normalize(&m.inverse_rotate(dir));

        let he      = self.half_extents();
        let cangtol = angtol.cos();
        let sangtol = angtol.sin();

        if na::dimension::<P>() == 1 {
            let p = self.support_point(m, dir);
            out_points.push(p);
            return 1;
        }
        else if na::dimension::<P>() == 2 {
            for i in 0usize .. 2 {
                let j = (i + 1) % 2;

                if na::abs(&local_dir[i]) > cangtol {
                    let mut p = na::origin::<P>() + *he;
                    if local_dir[i] < na::zero() {
                        p[i] = -p[i];
                    }

                    out_points.push(m.transform(&p));
                    p[j] = -p[j];
                    out_points.push(m.transform(&p));

                    return 2;
                }
            }
        }
        else {
            // Test faces for support points.
            for i in 0usize .. 3 {
                if na::abs(&local_dir[i]) > cangtol {
                    let j = (i + 1) % 3;
                    let k = (i + 2) % 3;

                    // Return the whole face.
                    let mut p = na::origin::<P>() + *he;

                    if local_dir[i] < na::zero() {
                        p[i] = -p[i];
                    }

                    out_points.push(m.transform(&p));

                    // Flip one component at a time.
                    p[j] = -p[j];
                    out_points.push(m.transform(&p));

                    p[k] = -p[k];
                    out_points.push(m.transform(&p));

                    p[j] = -p[j];
                    out_points.push(m.transform(&p));

                    return 4;
                }
            }

            // Test edges for support points.
            for i in 0usize .. 3 {
                if na::abs(&local_dir[i]) < sangtol {
                    let j = (i + 1) % 3;
                    let k = (i + 2) % 3;

                    // Return the edge parallel to the i-th direction.
                    let mut p = na::origin::<P>() + *he;

                    if local_dir[j] < na::zero() {
                        p[j] = -he[j];
                    }
                    if local_dir[k] < na::zero() {
                        p[k] = -he[k];
                    }

                    out_points.push(m.transform(&p));
                    p[i] = -p[i];
                    out_points.push(m.transform(&p));

                    return 2;
                }
            }
        }

        // The support point is only one vertex.
        let mut pres: P = na::origin();

        for i in 0usize .. na::dimension::<P>() {
            if local_dir[i] < na::zero() {
                pres[i] = -he[i];
            }
            else {
                pres[i] = he[i];
            }
        }

        out_points.push(m.transform(&pres));

        1
     }
}
