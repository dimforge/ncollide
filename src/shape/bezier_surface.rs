//! Bezier surface.

use std::iter::repeat;
use na::{Norm, RotationMatrix, Cross, Rotation, Mat, Rotate};
use na;
use procedural;
use utils::data::vec_slice::{VecSlice, VecSliceMut};
use shape::bezier_curve;
use math::{Scalar, Point, Vect};

/// Cache used to evaluate a bezier surface at a given parameter.
///
/// The same cache can be used for multiple evaluations (that is the whole point of this).
pub struct BezierSurfaceEvaluationCache<P> {
    u_cache: Vec<P>,
    v_cache: Vec<P>
}

/// Procedural generator of non-rational Bézier surfaces.
#[deriving(PartialEq, Show, Clone, RustcEncodable, RustcDecodable)]
pub struct BezierSurface<P> {
    control_points:     Vec<P>, // u-major storage.
    nupoints:           uint,
    nvpoints:           uint
}

impl<N, P: Point<N, V>, V> BezierSurface<P> {
    /// Creates a new procedural generator of non-rational bézier surfaces.
    ///
    /// # Parameters:
    /// * `control_points`: The control points of the bézier surface.
    /// * `nupoints`:       The number of control points per u-iso-curve.
    /// * `nvpoints`:       The number of control points per v-iso-curve.
    ///
    /// # Failures:
    /// Fails if the vector of control points does not contain exactly `nupoints * nvpoints`
    /// elements.
    pub fn new(control_points: Vec<P>, nupoints: uint, nvpoints: uint) -> BezierSurface<P> {
        assert!(nupoints * nvpoints == control_points.len());

        BezierSurface {
            control_points:     control_points,
            nupoints:           nupoints,
            nvpoints:           nvpoints
        }
    }

    /// Creates a new bézier curve of degrees `u_degree x v_degree`.
    ///
    /// Its control points are initialized to zero.
    pub fn new_with_degrees(degree_u: uint, degree_v: uint) -> BezierSurface<P> {
        let nupoints = degree_u + 1;
        let nvpoints = degree_v + 1;
        BezierSurface::new(repeat(na::orig()).take(nupoints * nvpoints).collect(), nupoints, nvpoints)
    }

    /// Creates a cache to avaluate a bezier surface.
    pub fn new_evaluation_cache() -> BezierSurfaceEvaluationCache<P> {
        BezierSurfaceEvaluationCache {
            u_cache: Vec::new(),
            v_cache: Vec::new()
        }
    }

    /// Changes the degree of this surface and reset its control points to zero.
    ///
    /// If the surface already has the specified degrees, nothing is changed, and `false` is
    /// returned. Otherwise `true` is returned.
    pub fn reset_with_degrees(&mut self, degree_u: uint, degree_v: uint) -> bool {
        let nupoints = degree_u + 1;
        let nvpoints = degree_v + 1;

        if self.nupoints == nupoints && self.nvpoints == nvpoints {
            false
        }
        else {
            self.control_points.clear();
            self.control_points.extend(repeat(na::orig()).take(nupoints * nvpoints));
            self.nupoints = nupoints;
            self.nvpoints = nvpoints;

            true
        }
    }

    /// The control points of this bézier surface.
    #[inline]
    pub fn control_points(&self) -> &[P] {
        self.control_points.as_slice()
    }

    /// The number of control points per u-iso-curve.
    #[inline]
    pub fn nupoints(&self) -> uint {
        self.nupoints
    }

    /// The number of control points per v-iso-curve.
    #[inline]
    pub fn nvpoints(&self) -> uint {
        self.nvpoints
    }

    /// The degree of u-iso-curves.
    #[inline]
    pub fn degree_u(&self) -> uint {
        self.nupoints - 1
    }

    /// The degree of v-iso-curves.
    #[inline]
    pub fn degree_v(&self) -> uint {
        self.nvpoints - 1
    }

    /// Get the `i`-th set of control points along the `u` parametric direction.
    #[inline]
    pub fn slice_u<'a>(&'a self, i: uint) -> VecSlice<'a, P> {
        VecSlice::new(self.control_points.slice_from(i * self.nupoints),
                      self.nupoints,
                      1)
    }

    /// Get the `i`-th set of control points along the `v` parametric direction.
    #[inline]
    pub fn slice_v<'a>(&'a self, i: uint) -> VecSlice<'a, P> {
        VecSlice::new(self.control_points.slice_from(i), self.nvpoints, self.nupoints)
    }

    /// Mutably get the `i`-th set of control points along the `u` parametric direction.
    #[inline]
    pub fn mut_slice_u<'a>(&'a mut self, i: uint) -> VecSliceMut<'a, P> {
        VecSliceMut::new(self.control_points.slice_from_mut(i * self.nupoints),
                         self.nupoints,
                         1)
    }

    /// Mutably get the `i`-th set of control points along the `v` parametric direction.
    #[inline]
    pub fn mut_slice_v<'a>(&'a mut self, i: uint) -> VecSliceMut<'a, P> {
        VecSliceMut::new(self.control_points.slice_from_mut(i), self.nvpoints, self.nupoints)
    }
}

impl<N, P, V> BezierSurface<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    /// Evaluates this bezier surface at the given parameter `t`.
    #[inline]
    pub fn at(&self, u: N, v: N, cache: &mut BezierSurfaceEvaluationCache<P>) -> P {
        procedural::bezier_surface_at(self.control_points.as_slice(),
                                      self.nupoints,
                                      self.nvpoints,
                                      u,
                                      v,
                                      &mut cache.u_cache,
                                      &mut cache.v_cache)
    }

    /// Subdivides this bezier at the given isocurve of parameter `u`.
    ///
    /// The union of the two resulting surfaces equals the original one.
    pub fn subdivide_u(&self, u: N, out1: &mut BezierSurface<P>, out2: &mut BezierSurface<P>) {
        out1.reset_with_degrees(self.degree_u(), self.degree_v());
        out2.reset_with_degrees(self.degree_u(), self.degree_v());

        for i in range(0, self.nvpoints) {
            bezier_curve::subdivide_bezier_curve_at(&self.slice_u(i), u,
                                                    &mut out1.mut_slice_u(i),
                                                    &mut out2.mut_slice_u(i));
        }
    }

    /// Subdivides this bezier at the given isocurve of parameter `v`.
    ///
    /// The union of the two resulting surfaces equals the original one.
    pub fn subdivide_v(&self, v: N, out1: &mut BezierSurface<P>, out2: &mut BezierSurface<P>) {
        out1.reset_with_degrees(self.degree_u(), self.degree_v());
        out2.reset_with_degrees(self.degree_u(), self.degree_v());

        for i in range(0, self.nupoints) {
            bezier_curve::subdivide_bezier_curve_at(&self.slice_v(i), v,
                                                    &mut out1.mut_slice_v(i),
                                                    &mut out2.mut_slice_v(i));
        }
    }

    /// Computes the derivative wrt. `u` of this bezier surface.
    pub fn diff_u(&self, out: &mut BezierSurface<P>) {
        out.reset_with_degrees(self.degree_u() - 1, self.degree_v());

        for i in range(0u, self.nvpoints) {
            bezier_curve::diff_bezier_curve(&self.slice_u(i), &mut out.mut_slice_u(i))
        }
    }

    /// Computes the derivative wrt. `v` of this bezier surface.
    pub fn diff_v(&self, out: &mut BezierSurface<P>) {
        out.reset_with_degrees(self.degree_u(), self.degree_v() - 1);

        for i in range(0u, self.nupoints) {
            bezier_curve::diff_bezier_curve(&self.slice_v(i), &mut out.mut_slice_v(i))
        }
    }

    /// Gets the endpoint at parameters `u = 0` and `v = 0`.
    #[inline]
    pub fn endpoint_00(&self) -> &P {
        &self.control_points[0]
    }

    /// Gets the endpoint at parameters `u = 0` and `v = 1`.
    #[inline]
    pub fn endpoint_01(&self) -> &P {
        &self.control_points[self.nupoints() - 1]
    }

    /// Gets the endpoint at parameters `u = 0` and `v = 0`.
    #[inline]
    pub fn endpoint_10(&self) -> &P {
        &self.control_points[self.control_points.len() - self.nupoints()]
    }

    /// Gets the endpoint at parameters `u = 1` and `v = 1`.
    #[inline]
    pub fn endpoint_11(&self) -> &P {
        self.control_points.last().unwrap()
    }
}

impl<N, P, V, AV, M> BezierSurface<P>
    where N: Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Cross<AV>,
          AV: Vect<N> + RotationMatrix<N, V, AV, M>,
          M:  Rotation<AV> + Rotate<V> + Mat<N, V, V> {
    /// Computes an infinite bounding cone of this surface.
    #[inline]
    pub fn bounding_cone_with_origin(&self, origin: &P) -> (V, N) {
        bounding_cone_with_origin(self.control_points.as_slice(), origin)
    }
}

/// Computes an infinite bounding cone of this surface.
pub fn bounding_cone_with_origin<N, P, V, AV, M>(points: &[P], origin: &P) -> (V, N)
    where N: Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Cross<AV>,
          AV: Vect<N> + RotationMatrix<N, V, AV, M>,
          M:  Rotation<AV> + Rotate<V> + Mat<N, V, V> {
    let mut axis  = points[0] - *origin;
    let mut theta = na::zero();

    // FIXME: this is not the best way to handle this (a better solution would be to loop on the
    // points until a non-zero vector is found).
    if na::is_zero(&axis.normalize()) {
        axis = na::zero();
        axis[0] = na::one();
    }

    for pt in points.slice_from(1).iter() {
        let mut dir = *pt - *origin;

        if !na::is_zero(&dir.normalize()) {
            let alpha = na::dot(&dir, &axis).acos();

            if alpha > theta {
                // We are outside of the bounding cone: enlarge it to wrap both the old bounding
                // cone and the new point.

                let mut rot_axis = na::cross(&dir, &axis);

                if !na::is_zero(&rot_axis.normalize()) {
                    let dangle = -(alpha - theta) * na::cast(0.5f64);
                    let rot    = (rot_axis * dangle).to_rot_mat();

                    axis  = na::rotate(&rot, &axis);
                    theta = (alpha + theta) * na::cast(0.5f64);
                }
                else {
                    // This happens if alpha ~= 0 or alpha ~= pi.
                    if alpha > na::one() { // NOTE: 1.0 is just a randomly chosen number in-between 0 and pi.
                        // alpha ~= pi
                        theta = alpha;
                    }
                    else {
                        // alpha ~= 0, do nothing.
                    }
                }
            }
        }
    }

    (axis, theta)
}
