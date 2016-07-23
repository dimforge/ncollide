//!
//! Support mapping based Cylinder shape.
//!

use std::f64;
use num::{Signed, Float};
use na::{Rotate, Transform, Norm};
use na;
use shape::SupportMap;
use math::{Scalar, Point, Vector};

/// SupportMap description of a cylinder shape with its principal axis aligned with the `y` axis.
#[derive(PartialEq, Debug, Clone, RustcEncodable, RustcDecodable)]
pub struct Cylinder<N> {
    half_height: N,
    radius:      N,
}

impl<N: Scalar> Cylinder<N> {
    /// Creates a new cylinder.
    ///
    /// # Arguments:
    /// * `half_height` - the half length of the cylinder along the `y` axis.
    /// * `radius` - the length of the cylinder along all other axis.
    pub fn new(half_height: N, radius: N) -> Cylinder<N> {
        assert!(half_height.is_positive() && radius.is_positive());

        Cylinder {
            half_height: half_height,
            radius:      radius
        }
    }

    /// The cylinder half length along the `y` axis.
    #[inline]
    pub fn half_height(&self) -> N {
        self.half_height.clone()
    }

    /// The radius of the cylinder along all but the `y` axis.
    #[inline]
    pub fn radius(&self) -> N {
        self.radius.clone()
    }
}

#[inline]
fn local_support_point<P: Point>(cyl: &Cylinder<<P::Vect as Vector>::Scalar>, local_dir: &P::Vect) -> P {
    let mut vres = local_dir.clone();

    let negative = local_dir[1].is_negative();

    vres[1] = na::zero();

    if na::is_zero(&vres.normalize_mut()) {
        vres = na::zero()
    }
    else {
        vres = vres * cyl.radius()
    }

    if negative {
        vres[1] = -cyl.half_height()
    }
    else {
        vres[1] = cyl.half_height()
    }

    na::origin::<P>() + vres
}


impl<P, M> SupportMap<P, M> for Cylinder<<P::Vect as Vector>::Scalar>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
    fn support_point(&self, m: &M, dir: &P::Vect) -> P {
        let local_dir     = m.inverse_rotate(dir);
        let local_supp_pt = local_support_point(self, &local_dir);

        m.transform(&local_supp_pt)
    }


    fn support_point_set(&self,
                         m:            &M,
                         dir:          &P::Vect,
                         angtol:       <P::Vect as Vector>::Scalar,
                         approx_count: usize,
                         out_points:   &mut Vec<P>)
                         -> usize {
        assert!(na::dimension::<P>() <= 3,
                "Set-valued support point computation not yet implemented for dimensions > 3.");

        let local_dir = na::normalize(&m.inverse_rotate(dir));

        let cangtol = angtol.cos();
        let sangtol = angtol.sin();

        if na::abs(&local_dir[1]) < sangtol {
            // Return a vertical line.
            let mut vres = local_dir.clone();

            vres[1] = na::zero();

            let _ = vres.normalize_mut();
            vres = vres * self.radius();

            let mut pres = na::origin::<P>() + vres;
            pres[1] = self.half_height();
            out_points.push(m.transform(&pres));

            pres[1] = -self.half_height();
            out_points.push(m.transform(&pres));

            2
        }
        else if na::abs(&local_dir[1]) > cangtol {
            if na::dimension::<P>() == 2 {
                let mut pres = na::origin::<P>();

                if local_dir[1].is_negative() {
                    pres[1] = -self.half_height();
                }
                else {
                    pres[1] = self.half_height();
                }

                pres[0] = self.radius();
                out_points.push(m.transform(&pres));

                pres[0] = -self.radius();
                out_points.push(m.transform(&pres));

                2
            }
            else {
                // Sample the circle.
                let ang_increment = 2.0 * f64::consts::PI / (approx_count as f64);
                let ang_increment: <P::Vect as Vector>::Scalar = na::cast(ang_increment);
                let is_negative   = local_dir[1].is_negative();

                let mut pres = na::origin::<P>();
                pres[1] = if is_negative { -self.half_height() } else { self.half_height() };

                for i in 0 .. approx_count {
                    let curr_param = ang_increment * na::cast(i as f64);
                    pres[0] = curr_param.cos();
                    pres[2] = curr_param.sin();

                    out_points.push(m.transform(&pres));
                }

                // Also add the actual support point.
                let mut vres = local_dir.clone();
                vres[1] = na::zero();

                if !na::is_zero(&vres.normalize_mut()) {
                    pres[0] = vres[0];
                    pres[2] = vres[2];
                    out_points.push(m.transform(&pres));
                }

                approx_count + 1
            }
        }
        else {
            // Only one point.
            let local_supp_pt = local_support_point(self, &local_dir);
            out_points.push(m.transform(&local_supp_pt));

            1
        }
     }
}
