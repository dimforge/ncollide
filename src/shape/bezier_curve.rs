//! Bezier curves.

use na;
use procedural;
use utils::data::vec_slice::{VecSlice, VecSliceMut};
use math::{Scalar, Point, Vect};

/// Cache used to evaluate a bezier curve at a given parameter.
///
/// The same cache can be used for multiple evaluations (that is the whole point of this).
pub struct BezierCurveEvaluationCache<P> {
    cache: Vec<P>,
}

/// Procedural generator of non-rational Bézier curve.
#[deriving(PartialEq, Show, Clone, RustcEncodable, RustcDecodable)]
pub struct BezierCurve<P> {
    control_points: Vec<P>
}

impl<N, P, V> BezierCurve<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    /// Creates a new non-rational bézier curve.
    ///
    /// # Parameters:
    /// * `control_points`: The control points of the bézier curve.
    pub fn new(control_points: Vec<P>) -> BezierCurve<P> {
        BezierCurve {
            control_points: control_points
        }
    }

    /// Creates a new bézier curve of degree `degree`.
    ///
    /// Its control points are initialized to zero.
    pub fn new_with_degree(degree: uint) -> BezierCurve<P> {
        BezierCurve::new(Vec::from_elem(degree + 1, na::orig()))
    }

    /// Creates a cache to avaluate a bezier curve.
    pub fn new_evaluation_cache() -> BezierCurveEvaluationCache<P> {
        BezierCurveEvaluationCache {
            cache: Vec::new()
        }
    }

    /// The control points of this bézier curve.
    #[inline]
    pub fn control_points(&self) -> &[P] {
        self.control_points.as_slice()
    }

    /// The control points of this bézier curve.
    #[inline]
    pub fn control_points_mut(&mut self) -> &mut [P] {
        self.control_points.as_mut_slice()
    }

    /// The number of control points per u-iso-curve.
    #[inline]
    pub fn npoints(&self) -> uint {
        self.control_points.len()
    }

    /// The degree of this curve.
    #[inline]
    pub fn degree(&self) -> uint {
        self.npoints() - 1
    }

    /// Evaluates this bezier curve at the given parameter `t`.
    #[inline]
    pub fn at(&self, t: N, cache: &mut BezierCurveEvaluationCache<P>) -> P {
        procedural::bezier_curve_at(self.control_points.as_slice(), t, &mut cache.cache)
    }

    /// Subdivides this bezier at the given parameter.
    ///
    /// The union of the two resulting curves equals the original one.
    pub fn subdivide_at(&self, t: N, out1: &mut BezierCurve<P>, out2: &mut BezierCurve<P>) {
        let mut slice_1    = VecSliceMut::new(out1.control_points_mut(), self.npoints(), 1);
        let mut slice_2    = VecSliceMut::new(out2.control_points_mut(), self.npoints(), 1);
        let control_points = VecSlice::new(self.control_points.as_slice(), self.npoints(), 1);
        subdivide_bezier_curve_at(&control_points, t, &mut slice_1, &mut slice_2);
    }

    /// Computes the derivative of this bezier curve.
    pub fn diff(&self, out: &mut BezierCurve<P>) {
        let outpoints      = out.npoints();
        let mut slice      = VecSliceMut::new(out.control_points_mut(), outpoints, 1);
        let control_points = VecSlice::new(self.control_points.as_slice(), self.npoints(), 1);

        diff_bezier_curve(&control_points, &mut slice)
    }
}

/// Subdivides the bezier curve with control points `control_points` at a given parameter `t`.
pub fn subdivide_bezier_curve_at<N, P, V>(control_points: &VecSlice<P>,
                                          t:              N,
                                          out_left:       &mut VecSliceMut<P>,
                                          out_right:      &mut VecSliceMut<P>)
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    assert!(control_points.len() == out_left.len(), "The result vector must have the same length as the input.");
    assert!(control_points.len() == out_right.len(), "The result vector must have the same length as the input.");

    let _1: N = na::one();
    let t_1   = _1 - t;

    out_right.copy_from(control_points);

    *out_left.get_mut(0) = control_points.get(0).clone();
    for i in range(1u, control_points.len()) {
        for j in range(0u, control_points.len() - i) {
            *out_right.get_mut(j) = *out_right.get(j) * t_1 + *out_right.get(j + 1).as_vec() * t;
        }
        *out_left.get_mut(i) = out_right.get(0).clone();
    }
}


/// Computes the control point of a bezier curve's derivative.
pub fn diff_bezier_curve<N, P, V>(control_points: &VecSlice<P>, out: &mut VecSliceMut<P>)
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    assert!(out.len() == control_points.len() - 1);

    let degree: N = na::cast(out.len() as f64);

    for i in range(0u, out.len()) {
        unsafe {
            *out.unsafe_get_mut(i) = na::orig::<P>() + (*control_points.unsafe_get(i + 1) - *control_points.unsafe_get(i)) * degree
        }
    }
}
