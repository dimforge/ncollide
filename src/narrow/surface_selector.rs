//! Heuristics to select surface containing the projection of a point.

use na;
use bounding_volume::{HasBoundingSphere, BoundingVolume, BoundingSphere};
use geom::BezierSurface;
use math::{Scalar, Vect};

/// Trait implemented by the heristics for surface selection during collision detection.
pub trait SurfaceSelector<D> {
    /// Sets the maximum local minimal distance.
    fn set_max_lmd(&mut self, max_lmd: Scalar);
    /// Tells whether a surface is flat enough to run a numerical resolution algorithm.
    fn is_flat(&mut self, surf: &BezierSurface, data: &D) -> bool;
    /// Tells whether a surface might contain a closest point or not.
    fn may_contain_a_closest_point(&mut self, pt: &Vect, b: &BezierSurface, data: &D) -> bool;
    /// Allocates data used to test flatnass and closest point containment.
    fn create_test_data(&mut self, b: &BezierSurface) -> D;
}

/// A selector that does not filter out any surface.
///
/// Do not use this.
#[deriving(Clone)]
pub struct YesSirSurfaceSelector;

impl YesSirSurfaceSelector {
    /// Creates a new `YesSirSurfaceSelector`.
    pub fn new() -> YesSirSurfaceSelector {
        YesSirSurfaceSelector
    }
}

impl SurfaceSelector<()> for YesSirSurfaceSelector {
    fn set_max_lmd(&mut self, _: Scalar) {
    }

    fn is_flat(&mut self, _: &BezierSurface, _: &()) -> bool {
        false
    }

    fn may_contain_a_closest_point(&mut self, _: &Vect, _: &BezierSurface, _: &()) -> bool {
        true
    }

    fn create_test_data(&mut self, _: &BezierSurface) -> () {
        ()
    }
}

/// A selector that tries to project the point on one of the extremal point of the surface.
///
/// This is the criteria from `Improved algorithms for the projection of points on NURBS curves and
/// surfaces`, Ilijas Selimovic.
#[deriving(Clone)]
pub struct HyperPlaneSurfaceSelector {
    max_lmd: Scalar
}

impl HyperPlaneSurfaceSelector {
    /// Creates a new hyperplane-based surface selector.
    pub fn new(max_lmd: Scalar) -> HyperPlaneSurfaceSelector {
        HyperPlaneSurfaceSelector {
            max_lmd: max_lmd
        }
    }
}

impl SurfaceSelector<BoundingSphere> for HyperPlaneSurfaceSelector {
    fn set_max_lmd(&mut self, max_lmd: Scalar) {
        self.max_lmd = max_lmd
    }

    fn is_flat(&mut self, _: &BezierSurface, _: &BoundingSphere) -> bool {
        false
    }

    fn may_contain_a_closest_point(&mut self, pt: &Vect, b: &BezierSurface, bs: &BoundingSphere) -> bool {
        if bs.intersects(&BoundingSphere::new(pt.clone(), self.max_lmd)) {
            let endpoints = [ b.endpoint_01(), b.endpoint_10(), b.endpoint_11() ];

            let mut closest_sqnorm   = na::sqnorm(&(*b.endpoint_00() - *pt));
            let mut closest_endpoint = b.endpoint_00();

            for endpoint in endpoints.iter() {
                let sqnorm = na::sqnorm(&(*endpoint - *pt));

                if sqnorm < closest_sqnorm {
                    closest_endpoint = *endpoint;
                    closest_sqnorm   = sqnorm;
                }
            }

            let pt_endpoint = *closest_endpoint - *pt;

            for control_point in b.control_points().iter() {
                if na::dot(&(*control_point - *closest_endpoint), &pt_endpoint) <= na::zero() {
                    return true
                }
            }
        }

        false
    }

    fn create_test_data(&mut self, b: &BezierSurface) -> BoundingSphere {
        b.bounding_sphere(&na::one()) // FIXME: pass Identity::new() ?
    }

}

/// A selector that tests the orthogonality condition using the surface tangent cone.
///
/// This is the criteria from `Distance Extrema for Spline Models Using Tangent Cones`, Ilijas
/// Selimovic.
#[deriving(Clone)]
pub struct TangentConesSurfaceSelector {
    diff_u:  BezierSurface,
    diff_v:  BezierSurface,
    max_lmd: Scalar
}

impl TangentConesSurfaceSelector {
    /// Creates a new tangent-cone based surface detector. 
    pub fn new(max_lmd: Scalar) -> TangentConesSurfaceSelector {
        TangentConesSurfaceSelector {
            diff_u:  BezierSurface::new_with_degrees(0, 0),
            diff_v:  BezierSurface::new_with_degrees(0, 0),
            max_lmd: max_lmd
        }
    }
}

/// Data used by the `TangentconesSurfaceSelector` to test orthogonality and flatness.
pub struct TangentConesSurfaceSelectorTestData {
    bounding_sphere: BoundingSphere,
    spread_u:        Scalar,
    spread_v:        Scalar,
    axis_u:          Vect,
    axis_v:          Vect
}

impl SurfaceSelector<TangentConesSurfaceSelectorTestData> for TangentConesSurfaceSelector {
    fn set_max_lmd(&mut self, max_lmd: Scalar) {
        self.max_lmd = max_lmd
    }

    fn is_flat(&mut self, _: &BezierSurface, _: &TangentConesSurfaceSelectorTestData) -> bool {
        false // XXX: we could be smarter here
    }

    fn may_contain_a_closest_point(&mut self,
                                   pt: &Vect,
                                   b:  &BezierSurface,
                                   d:  &TangentConesSurfaceSelectorTestData)
                                   -> bool {
        if d.bounding_sphere.intersects(&BoundingSphere::new(pt.clone(), self.max_lmd)) {
            let (axis, ang)   = b.bounding_cone_with_origin(pt);

            let ang_with_u_axis = na::dot(&axis, &d.axis_u).acos();
            let ang_with_v_axis = na::dot(&axis, &d.axis_v).acos();

            ang_with_u_axis - ang - d.spread_u <= Float::frac_pi_2() &&
            ang_with_u_axis + ang + d.spread_u >= Float::frac_pi_2() &&
            ang_with_v_axis - ang - d.spread_v <= Float::frac_pi_2() &&
            ang_with_v_axis + ang + d.spread_v >= Float::frac_pi_2()
        }
        else {
            false
        }
    }

    fn create_test_data(&mut self, b: &BezierSurface) -> TangentConesSurfaceSelectorTestData {
        let bounding_sphere = b.bounding_sphere(&na::one()); // FIXME: pass Identity::new() ?

        b.diff_u(&mut self.diff_u);
        b.diff_v(&mut self.diff_v);

        let (axis_u, spread_u) = self.diff_u.bounding_cone_with_origin(&na::zero());
        let (axis_v, spread_v) = self.diff_v.bounding_cone_with_origin(&na::zero());

        TangentConesSurfaceSelectorTestData {
            bounding_sphere: bounding_sphere,
            spread_u:        spread_u,
            spread_v:        spread_v,
            axis_u:          axis_u,
            axis_v:          axis_v
        }
    }
}

