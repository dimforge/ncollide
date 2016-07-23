//! Support mapping based Cone shape.

use std::f64;
use num::{Signed, Float};
use na::{Rotate, Transform, Norm};
use na;
use shape::SupportMap;
use math::{Scalar, Point, Vector};

/// SupportMap description of a cylinder shape with its principal axis aligned with the `y` axis.
#[derive(PartialEq, Debug, Clone, RustcEncodable, RustcDecodable)]
pub struct Cone<N> {
    half_height: N,
    radius:      N,
}

impl<N> Cone<N>
    where N: Scalar {
    /// Creates a new cone.
    ///
    /// # Arguments:
    /// * `half_height` - the half length of the cone along the `y` axis.
    /// * `radius` - the length of the cone along all other axis.
    pub fn new(half_height: N, radius: N) -> Cone<N> {
        assert!(half_height.is_positive() && radius.is_positive());

        Cone {
            half_height: half_height,
            radius:      radius
        }
    }

    /// The cone half length along the `y` axis.
    #[inline]
    pub fn half_height(&self) -> N {
        self.half_height.clone()
    }

    /// The radius of the cone along all but the `y` axis.
    #[inline]
    pub fn radius(&self) -> N {
        self.radius.clone()
    }

    /// The half-angle between the cone symmetry axis and its contour.
    #[inline]
    pub fn apex_half_angle(&self) -> N {
        self.radius().atan2(self.half_height() * na::cast(2.0))
    }
}

#[inline]
fn local_support_point<P: Point>(cone: &Cone<<P::Vect as Vector>::Scalar>, local_dir: &P::Vect) -> P {
    let mut vres = local_dir.clone();

    vres[1] = na::zero();

    if na::is_zero(&vres.normalize_mut()) {
        vres = na::zero();

        if local_dir[1].is_negative() {
            vres[1] = -cone.half_height()
        }
        else {
            vres[1] = cone.half_height()
        }
    }
    else {
        vres = vres * cone.radius();
        vres[1] = -cone.half_height();

        if na::dot(local_dir, &vres) < local_dir[1] * cone.half_height() {
            vres = na::zero();
            vres[1] = cone.half_height()
        }
    }

    na::origin::<P>() + vres
}


impl<P, M> SupportMap<P, M> for Cone<<P::Vect as Vector>::Scalar>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
    #[inline]
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

        // FIXME: could be avoid having to compute the apex angle (and all those cosinuses too)?
        let apex_hang     = self.apex_half_angle();
        let cangtol       = angtol.cos();
        let side_ang      = -apex_hang + na::cast(f64::consts::FRAC_PI_2);
        let sangtol_side1 = (side_ang - angtol).cos();
        let sangtol_side2 = (side_ang + angtol).cos();

        if local_dir[1] > sangtol_side2 && local_dir[1] < sangtol_side1 {
            // Return a vertical line.
            let mut vres = local_dir.clone();

            vres[1] = na::zero();

            let _ = vres.normalize_mut();
            vres = vres * self.radius();

            let mut pres = na::origin::<P>();
            pres[1] = self.half_height();
            out_points.push(m.transform(&pres));

            let mut pres = na::origin::<P>() + vres;
            pres[1] = -self.half_height();
            out_points.push(m.transform(&pres));

            2
        }
        else if local_dir[1] < -cangtol {
            if na::dimension::<P>() == 2 {
                let mut pres = na::origin::<P>();

                pres[1] = -self.half_height();

                pres[0] = self.radius();
                out_points.push(m.transform(&pres));

                pres[0] = -self.radius();
                out_points.push(m.transform(&pres));

                2
            }
            else {
                // Sample the circle.
                // NOTE: refactor this (almost the same code as for the cylinder).
                let ang_increment = 2.0 * f64::consts::PI / (approx_count as f64);
                let ang_increment: <P::Vect as Vector>::Scalar = na::cast(ang_increment);

                let mut pres = na::origin::<P>();
                pres[1] = -self.half_height();

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
