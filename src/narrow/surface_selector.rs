//! Heuristics to select surface containing the projection of a point.

use na::{Rotation, RotationMatrix, Rotate, Translate, Cross, Mat, Identity};
use na;
use bounding_volume::{HasBoundingSphere, BoundingSphere};
use bounding_volume::BoundingVolume;
use geom::BezierSurface;
use math::{Scalar, Point, Vect};


/// Trait implemented by the heristics for surface selection during collision detection.
pub trait SurfaceSelector<N, P, D> {
    /// Sets the maximum local minimal distance.
    fn set_max_lmd(&mut self, max_lmd: N);
    /// Tells whether a surface is flat enough to run a numerical resolution algorithm.
    fn is_flat(&mut self, surf: &BezierSurface<P>, data: &D) -> bool;
    /// Tells whether a surface might contain a closest point or not.
    fn may_contain_a_closest_point(&mut self, pt: &P, b: &BezierSurface<P>, data: &D) -> bool;
    /// Allocates data used to test flatnass and closest point containment.
    fn create_test_data(&mut self, b: &BezierSurface<P>) -> D;
}

/// A selector that does not filter out any surface.
///
/// Do not use this.
#[deriving(Clone)]
pub struct YesSirSurfaceSelector<N, P>;

impl<N, P> YesSirSurfaceSelector<N, P> {
    /// Creates a new `YesSirSurfaceSelector`.
    pub fn new() -> YesSirSurfaceSelector<N, P> {
        YesSirSurfaceSelector
    }
}

impl<N, P> SurfaceSelector<N, P, ()> for YesSirSurfaceSelector<N, P> {
    fn set_max_lmd(&mut self, _: N) {
    }

    fn is_flat(&mut self, _: &BezierSurface<P>, _: &()) -> bool {
        false
    }

    fn may_contain_a_closest_point(&mut self, _: &P, _: &BezierSurface<P>, _: &()) -> bool {
        true
    }

    fn create_test_data(&mut self, _: &BezierSurface<P>) -> () {
        ()
    }
}

/// A selector that tries to project the point on one of the extremal point of the surface.
///
/// This is the criteria from `Improved algorithms for the projection of points on NURBS curves and
/// surfaces`, Ilijas Selimovic.
#[deriving(Clone)]
pub struct HyperPlaneSurfaceSelector<N, P> {
    max_lmd: N
}

impl<N, P> HyperPlaneSurfaceSelector<N, P> {
    /// Creates a new hyperplane-based surface selector.
    pub fn new(max_lmd: N) -> HyperPlaneSurfaceSelector<N, P> {
        HyperPlaneSurfaceSelector {
            max_lmd: max_lmd
        }
    }
}

impl<N, P, V> SurfaceSelector<N, P, BoundingSphere<N, P>> for HyperPlaneSurfaceSelector<N, P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P> {
    fn set_max_lmd(&mut self, max_lmd: N) {
        self.max_lmd = max_lmd
    }

    fn is_flat(&mut self, _: &BezierSurface<P>, _: &BoundingSphere<N, P>) -> bool {
        false
    }

    fn may_contain_a_closest_point(&mut self, pt: &P, b: &BezierSurface<P>, bs: &BoundingSphere<N, P>) -> bool {
        if bs.intersects(&BoundingSphere::new(pt.clone(), self.max_lmd)) {
            let endpoints = [ b.endpoint_01(), b.endpoint_10(), b.endpoint_11() ];

            let mut closest_sqnorm   = na::sqnorm(&(*b.endpoint_00() - *pt));
            let mut closest_endpoint = b.endpoint_00();

            for endpoint in endpoints.iter() {
                let sqnorm = na::sqnorm(&(**endpoint - *pt));

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

    fn create_test_data(&mut self, b: &BezierSurface<P>) -> BoundingSphere<N, P> {
        b.bounding_sphere(&Identity::new())
    }
}

/// A selector that tests the orthogonality condition using the surface tangent cone.
///
/// This is the criteria from `Distance Extrema for Spline Models Using Tangent Cones`, Ilijas
/// Selimovic.
#[deriving(Clone)]
pub struct TangentConesSurfaceSelector<N, P> {
    diff_u:  BezierSurface<P>,
    diff_v:  BezierSurface<P>,
    max_lmd: N
}

impl<N, P, V> TangentConesSurfaceSelector<N, P>
    where N: Scalar,
          P: Point<N, V> {
    /// Creates a new tangent-cone based surface detector.
    pub fn new(max_lmd: N) -> TangentConesSurfaceSelector<N, P> {
        TangentConesSurfaceSelector {
            diff_u:  BezierSurface::new_with_degrees(0, 0),
            diff_v:  BezierSurface::new_with_degrees(0, 0),
            max_lmd: max_lmd
        }
    }
}

/// Data used by the `TangentconesSurfaceSelector` to test orthogonality and flatness.
pub struct TangentConesSurfaceSelectorTestData<N, P, V> {
    bounding_sphere: BoundingSphere<N, P>,
    spread_u:        N,
    spread_v:        N,
    axis_u:          V,
    axis_v:          V
}

impl<N, P, V, AV, M> SurfaceSelector<N, P, TangentConesSurfaceSelectorTestData<N, P, V>> for TangentConesSurfaceSelector<N, P>
    where N: Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P> + Cross<AV>,
          AV: Vect<N> + RotationMatrix<N, V, AV, M>,
          M:  Rotation<AV> + Rotate<V> + Mat<N, V, V> {
    fn set_max_lmd(&mut self, max_lmd: N) {
        self.max_lmd = max_lmd
    }

    fn is_flat(&mut self, _: &BezierSurface<P>, _: &TangentConesSurfaceSelectorTestData<N, P, V>) -> bool {
        false // XXX: we could be smarter here
    }

    fn may_contain_a_closest_point(&mut self,
                                   pt: &P,
                                   b:  &BezierSurface<P>,
                                   d:  &TangentConesSurfaceSelectorTestData<N, P, V>)
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

    fn create_test_data(&mut self, b: &BezierSurface<P>) -> TangentConesSurfaceSelectorTestData<N, P, V> {
        let bounding_sphere = b.bounding_sphere(&Identity::new());

        b.diff_u(&mut self.diff_u);
        b.diff_v(&mut self.diff_v);

        let (axis_u, spread_u) = self.diff_u.bounding_cone_with_origin(&na::orig());
        let (axis_v, spread_v) = self.diff_v.bounding_cone_with_origin(&na::orig());

        TangentConesSurfaceSelectorTestData {
            bounding_sphere: bounding_sphere,
            spread_u:        spread_u,
            spread_v:        spread_v,
            axis_u:          axis_u,
            axis_v:          axis_v
        }
    }
}

