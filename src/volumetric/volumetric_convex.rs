// XXX: implement this for 2d too.

#[cfg(feature = "3d")]
use na::Mat3;
#[cfg(feature = "3d")]
use na;
#[cfg(feature = "3d")]
use utils;
#[cfg(feature = "3d")]
use procedural::{SplitIndexBuffer, UnifiedIndexBuffer};
#[cfg(feature = "3d")]
use geom::Convex;
#[cfg(feature = "3d")]
use volumetric::Volumetric;
#[cfg(feature = "3d")]
use math::{Scalar, Vect, AngularInertia};
#[cfg(feature = "3d")]
use std::num::Zero;

#[cfg(feature = "3d")]
fn tetrahedron_unit_inertia_tensor_wrt_point(point: &Vect, p1: &Vect, p2: &Vect, p3: &Vect, p4: &Vect) -> AngularInertia {
    let p1 = *p1 - *point;
    let p2 = *p2 - *point;
    let p3 = *p3 - *point;
    let p4 = *p4 - *point;

    let _frac_10: Scalar = na::cast(0.1f64);
    let _frac_20: Scalar = na::cast(0.05f64);
    let _2      : Scalar = na::cast(2.0f64);

    // Just for readability.
    let x1 = p1.x; let y1 = p1.y; let z1 = p1.z;
    let x2 = p2.x; let y2 = p2.y; let z2 = p2.z;
    let x3 = p3.x; let y3 = p3.y; let z3 = p3.z;
    let x4 = p4.x; let y4 = p4.y; let z4 = p4.z;

    let diag_x = x1 * x1 + x1 * x2 + x2 * x2 + x1 * x3 + x2 * x3 + x3 * x3 + x1 * x4 + x2 * x4 + x3 * x4 + x4 * x4;
    let diag_y = y1 * y1 + y1 * y2 + y2 * y2 + y1 * y3 + y2 * y3 + y3 * y3 + y1 * y4 + y2 * y4 + y3 * y4 + y4 * y4;
    let diag_z = z1 * z1 + z1 * z2 + z2 * z2 + z1 * z3 + z2 * z3 + z3 * z3 + z1 * z4 + z2 * z4 + z3 * z4 + z4 * z4;

    let a0 = (diag_y + diag_z) * _frac_10;
    let b0 = (diag_z + diag_x) * _frac_10;
    let c0 = (diag_x + diag_y) * _frac_10;

    let a1 = (y1 * z1 * _2 + y2 * z1      + y3 * z1      + y4 * z1 +
              y1 * z2      + y2 * z2 * _2 + y3 * z2      + y4 * z2 +
              y1 * z3      + y2 * z3      + y3 * z3 * _2 + y4 * z3 +
              y1 * z4      + y2 * z4      + y3 * z4      + y4 * z4 * _2) * _frac_20;
    let b1 = (x1 * z1 * _2 + x2 * z1      + x3 * z1      + x4 * z1 +
              x1 * z2      + x2 * z2 * _2 + x3 * z2      + x4 * z2 +
              x1 * z3      + x2 * z3      + x3 * z3 * _2 + x4 * z3 +
              x1 * z4      + x2 * z4      + x3 * z4      + x4 * z4 * _2) * _frac_20;
    let c1 = (x1 * y1 * _2 + x2 * y1      + x3 * y1      + x4 * y1 +
              x1 * y2      + x2 * y2 * _2 + x3 * y2      + x4 * y2 +
              x1 * y3      + x2 * y3      + x3 * y3 * _2 + x4 * y3 +
              x1 * y4      + x2 * y4      + x3 * y4      + x4 * y4 * _2) * _frac_20;

    Mat3::new(
        a0, -b1, -c1,
        -b1, b0, -a1,
        -c1, -a1, c0
    )
}

#[cfg(feature = "3d")]
pub fn convex_volume_and_center(convex: &Convex) -> (Scalar, Vect) {
    let geometric_center = utils::center(convex.pts());

    let mut res = na::zero::<Vect>();
    let mut vol = na::zero::<Scalar>();

    match convex.mesh().indices {
        UnifiedIndexBuffer(ref idx) => {
            for t in idx.iter() {
                let p2 = &convex.mesh().coords[t.x as uint];
                let p3 = &convex.mesh().coords[t.y as uint];
                let p4 = &convex.mesh().coords[t.z as uint];

                let volume = utils::tetrahedron_volume(&geometric_center, p2, p3, p4);
                let center = utils::tetrahedron_center(&geometric_center, p2, p3, p4);

                res = res + center * volume;
                vol = vol + volume;
            }
        },
        SplitIndexBuffer(_) => unreachable!()
    }

    if vol.is_zero() {
        (vol, geometric_center)
    }
    else {
        (vol, res / vol)
    }
}

#[cfg(feature = "3d")]
impl Volumetric for Convex {
    fn surface(&self) -> Scalar {
        let mut surface = na::zero::<Scalar>();

        match self.mesh().indices {
            UnifiedIndexBuffer(ref idx) => {
                for t in idx.iter() {
                    let p1 = &self.mesh().coords[t.x as uint];
                    let p2 = &self.mesh().coords[t.y as uint];
                    let p3 = &self.mesh().coords[t.z as uint];

                    surface = surface + utils::triangle_area(p1, p2, p3);
                }
            },
            SplitIndexBuffer(_) => unreachable!()
        }

        surface
    }

    fn volume(&self) -> Scalar {
        convex_volume_and_center(self).val0()
    }

    fn center_of_mass(&self) -> Vect {
        convex_volume_and_center(self).val1()
    }

    fn unit_angular_inertia(&self) -> AngularInertia {
        let (volume, _, i) = self.mass_properties(&na::one());

        if volume.is_zero() {
            na::zero()
        }
        else {
            i / volume
        }
    }

    fn mass_properties(&self, density: &Scalar) -> (Scalar, Vect, AngularInertia) {
        let (volume, com) = convex_volume_and_center(self);

        let mut itot = na::zero::<AngularInertia>();

        match self.mesh().indices {
            UnifiedIndexBuffer(ref idx) => {
                for t in idx.iter() {
                    let p2 = &self.mesh().coords[t.x as uint];
                    let p3 = &self.mesh().coords[t.y as uint];
                    let p4 = &self.mesh().coords[t.z as uint];

                    let vol   = utils::tetrahedron_volume(&com, p2, p3, p4);
                    let ipart = tetrahedron_unit_inertia_tensor_wrt_point(&com, &com, p2, p3, p4);

                    itot = itot + ipart * vol;
                }
            },
            SplitIndexBuffer(_) => unreachable!()
        }

        (volume * *density, com, itot * *density)
    }
}

#[cfg(test)]
mod test {
    use na::Vec3;
    use na;
    use geom::{Convex, Cuboid};
    use procedural;
    use volumetric::Volumetric;

    #[test]
    #[cfg(feature = "3d")]
    fn test_inertia_tensor() {
        let excentricity = 10.0;

        let mut shape = procedural::cuboid(&Vec3::new(2.0 - 0.08, 2.0 - 0.08, 2.0 - 0.08));

        for c in shape.coords.iter_mut() {
            c.x = c.x + excentricity;
            c.y = c.y + excentricity;
            c.z = c.z + excentricity;
        }

        let actual   = Convex::new(shape.coords.as_slice()).unit_angular_inertia();
        let expected = Cuboid::new(Vec3::new(0.96, 0.96, 0.96)).unit_angular_inertia();

        assert!(na::approx_eq(&actual, &expected),
                format!("Inertia tensors do not match: actual {}, expected: {}.", actual, expected));
    }
}
