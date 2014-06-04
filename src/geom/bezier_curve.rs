use nalgebra::na;
use data::vec_slice::{VecSlice, VecSliceMut};
use math::{Vect, Scalar};

/// Cache used to evaluate a bezier curve at a given parameter.
///
/// The same cache can be used for multiple evaluations (that is the whole point of this).
pub struct BezierCurveEvaluationCache {
    cache: Vec<Vect>,
}

/// Procedural generator of non-rational Bézier curve.
#[deriving(Clone)]
pub struct BezierCurve {
    control_points: Vec<Vect> // u-major storage.
}

impl BezierCurve {
    /// Creates a new non-rational bézier curve.
    ///
    /// # Parameters:
    /// * `control_points`: The control points of the bézier curve.
    pub fn new(control_points: Vec<Vect>) -> BezierCurve {
        BezierCurve {
            control_points: control_points
        }
    }

    /// Creates a new bézier curve of degree `degree`.
    ///
    /// Its control points are initialized to zero.
    pub fn new_with_degree(degree: uint) -> BezierCurve {
        BezierCurve::new(Vec::from_elem(degree + 1, na::zero()))
    }

    /// Creates a cache to avaluate a bezier curve.
    pub fn new_evaluation_cache() -> BezierCurveEvaluationCache {
        BezierCurveEvaluationCache {
            cache: Vec::new()
        }
    }

    /// The control points of this bézier curve.
    #[inline]
    pub fn control_points<'a>(&'a self) -> &'a [Vect] {
        self.control_points.as_slice()
    }

    /// The control points of this bézier curve.
    #[inline]
    pub fn control_points_mut<'a>(&'a mut self) -> &'a mut [Vect] {
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
    pub fn at(&self, t: &Scalar, cache: &mut BezierCurveEvaluationCache) -> Vect {
        bezier_curve_at(self.control_points.as_slice(), t, &mut cache.cache)
    }

    /// Subdivides this bezier at the given parameter.
    ///
    /// The union of the two resulting curves equals the original one.
    pub fn subdivide_at(&self, t: &Scalar, out1: &mut BezierCurve, out2: &mut BezierCurve) {
        let mut slice_1    = VecSliceMut::new(out1.control_points_mut(), self.npoints(), 1);
        let mut slice_2    = VecSliceMut::new(out2.control_points_mut(), self.npoints(), 1);
        let control_points = VecSlice::new(self.control_points.as_slice(), self.npoints(), 1);
        subdivide_bezier_curve_at(&control_points, t, &mut slice_1, &mut slice_2);
    }

    /// Computes the derivative of this bezier curve.
    pub fn diff(&self, out: &mut BezierCurve) {
        let outpoints      = out.npoints();
        let mut slice      = VecSliceMut::new(out.control_points_mut(), outpoints, 1);
        let control_points = VecSlice::new(self.control_points.as_slice(), self.npoints(), 1);

        diff_bezier_curve(&control_points, &mut slice)
    }
}


/// Evaluates the bezier curve with control points `control_points`.
pub fn bezier_curve_at(control_points: &[Vect], t: &Scalar, cache: &mut Vec<Vect>) -> Vect {
    // De-Casteljau algorithm.
    cache.grow_set(control_points.len() - 1, &na::zero(), na::zero());

    let cache = cache.as_mut_slice();

    let _1: Scalar = na::cast(1.0);
    let t_1   = _1 - *t;

    unsafe {
        cache.as_mut_slice().copy_memory(control_points);
    }

    for i in range(1u, control_points.len()) {
        for j in range(0u, control_points.len() - i) {
            cache[j] = cache[j] * t_1 + cache[j + 1] * *t;
        }
    }

    cache[0].clone()
}

/// Subdivides the bezier curve with control points `control_points` at a given parameter `t`.
pub fn subdivide_bezier_curve_at(control_points: &VecSlice<Vect>,
                                 t:              &Scalar,
                                 out_left:       &mut VecSliceMut<Vect>,
                                 out_right:      &mut VecSliceMut<Vect>) {
    assert!(control_points.len() == out_left.len(), "The result vector must have the same length as the input.");
    assert!(control_points.len() == out_right.len(), "The result vector must have the same length as the input.");

    let _1: Scalar = na::cast(1.0);
    let t_1   = _1 - *t;

    out_right.copy_from(control_points);

    *out_left.get_mut(0) = control_points.get(0).clone();
    for i in range(1u, control_points.len()) {
        for j in range(0u, control_points.len() - i) {
            *out_right.get_mut(j) = *out_right.get(j) * t_1 + *out_right.get(j + 1) * *t;
        }
        *out_left.get_mut(i) = out_right.get(0).clone();
    }
}


/// Computes the control point of a bezier curve's derivative.
pub fn diff_bezier_curve(control_points: &VecSlice<Vect>, out: &mut VecSliceMut<Vect>) {
    assert!(out.len() == control_points.len() - 1);

    let degree: Scalar = na::cast(out.len());

    for i in range(0u, out.len()) {
        unsafe {
            *out.unsafe_get_mut(i) = (*control_points.unsafe_get(i + 1) - *control_points.unsafe_get(i)) * degree
        }
    }
}
