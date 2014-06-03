use std::num::Zero;
use nalgebra::na::{Norm, Indexable};
use nalgebra::na;
use util::vec_slice::{VecSlice, VecSliceMut};
use geom::bezier_curve;
use math::{Vect, Scalar};

#[cfg(not(dim4))]
use math::RotationMatrix;

#[cfg(dim4)]
use bounding_volume;

/// Cache used to evaluate a bezier surface at a given parameter.
///
/// The same cache can be used for multiple evaluations (that is the whole point of this).
pub struct BezierSurfaceEvaluationCache {
    u_cache: Vec<Vect>,
    v_cache: Vec<Vect>
}

/// Procedural generator of non-rational Bézier surfaces.
#[deriving(Clone)]
pub struct BezierSurface {
    control_points:     Vec<Vect>, // u-major storage.
    nupoints:           uint,
    nvpoints:           uint
}

impl BezierSurface {
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
    pub fn new(control_points: Vec<Vect>, nupoints: uint, nvpoints: uint) -> BezierSurface {
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
    pub fn new_with_degrees(degree_u: uint, degree_v: uint) -> BezierSurface {
        let nupoints = degree_u + 1;
        let nvpoints = degree_v + 1;
        BezierSurface::new(Vec::from_elem(nupoints * nvpoints, na::zero()), nupoints, nvpoints)
    }

    /// Creates a cache to avaluate a bezier surface.
    pub fn new_evaluation_cache() -> BezierSurfaceEvaluationCache {
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
            self.control_points.grow_set(nupoints * nvpoints - 1, &na::zero(), na::zero());
            self.nupoints = nupoints;
            self.nvpoints = nvpoints;

            true
        }
    }

    /// The control points of this bézier surface.
    #[inline]
    pub fn control_points<'a>(&'a self) -> &'a [Vect] {
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
    pub fn slice_u<'a>(&'a self, i: uint) -> VecSlice<'a, Vect> {
        VecSlice::new(self.control_points.slice_from(i * self.nupoints),
                      self.nupoints,
                      1)
    }

    /// Get the `i`-th set of control points along the `v` parametric direction.
    #[inline]
    pub fn slice_v<'a>(&'a self, i: uint) -> VecSlice<'a, Vect> {
        VecSlice::new(self.control_points.slice_from(i), self.nvpoints, self.nupoints)
    }

    /// Mutably get the `i`-th set of control points along the `u` parametric direction.
    #[inline]
    pub fn mut_slice_u<'a>(&'a mut self, i: uint) -> VecSliceMut<'a, Vect> {
        VecSliceMut::new(self.control_points.mut_slice_from(i * self.nupoints),
                         self.nupoints,
                         1)
    }

    /// Mutably get the `i`-th set of control points along the `v` parametric direction.
    #[inline]
    pub fn mut_slice_v<'a>(&'a mut self, i: uint) -> VecSliceMut<'a, Vect> {
        VecSliceMut::new(self.control_points.mut_slice_from(i), self.nvpoints, self.nupoints)
    }
}

impl BezierSurface {
    /// Evaluates this bezier surface at the given parameter `t`.
    #[inline]
    pub fn at(&self, u: &Scalar, v: &Scalar, cache: &mut BezierSurfaceEvaluationCache) -> Vect {
        bezier_surface_at(self.control_points.as_slice(),
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
    pub fn subdivide_u(&self, u: &Scalar, out1: &mut BezierSurface, out2: &mut BezierSurface) {
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
    pub fn subdivide_v(&self, v: &Scalar, out1: &mut BezierSurface, out2: &mut BezierSurface) {
        out1.reset_with_degrees(self.degree_u(), self.degree_v());
        out2.reset_with_degrees(self.degree_u(), self.degree_v());

        for i in range(0, self.nupoints) {
            bezier_curve::subdivide_bezier_curve_at(&self.slice_v(i), v,
                                                    &mut out1.mut_slice_v(i),
                                                    &mut out2.mut_slice_v(i));
        }
    }

    /// Computes the derivative wrt. `u` of this bezier surface.
    pub fn diff_u(&self, out: &mut BezierSurface) {
        out.reset_with_degrees(self.degree_u() - 1, self.degree_v());

        for i in range(0u, self.nvpoints) {
            bezier_curve::diff_bezier_curve(&self.slice_u(i), &mut out.mut_slice_u(i))
        }
    }

    /// Computes the derivative wrt. `v` of this bezier surface.
    pub fn diff_v(&self, out: &mut BezierSurface) {
        out.reset_with_degrees(self.degree_u(), self.degree_v() - 1);

        for i in range(0u, self.nupoints) {
            bezier_curve::diff_bezier_curve(&self.slice_v(i), &mut out.mut_slice_v(i))
        }
    }

    /// Gets the endpoint at parameters `u = 0` and `v = 0`.
    pub fn endpoint_00<'a>(&'a self) -> &'a Vect {
        self.control_points.get(0)
    }

    /// Gets the endpoint at parameters `u = 0` and `v = 1`.
    pub fn endpoint_01<'a>(&'a self) -> &'a Vect {
        self.control_points.get(self.nupoints() - 1)
    }

    /// Gets the endpoint at parameters `u = 0` and `v = 0`.
    pub fn endpoint_10<'a>(&'a self) -> &'a Vect {
        self.control_points.get(self.control_points.len() - self.nupoints())
    }

    /// Gets the endpoint at parameters `u = 1` and `v = 1`.
    pub fn endpoint_11<'a>(&'a self) -> &'a Vect {
        self.control_points.last().unwrap()
    }

    /// Computes an infinite bounding cone of this surface.
    pub fn bounding_cone_with_origin(&self, origin: &Vect) -> (Vect, Scalar) {
        bounding_cone_with_origin(self.control_points.as_slice(), origin)
    }
}

/// Evaluates the bezier curve with control points `control_points`.
pub fn bezier_surface_at(control_points: &[Vect],
                         nupoints:       uint,
                         nvpoints:       uint,
                         u:              &Scalar,
                         v:              &Scalar,
                         ucache:         &mut Vec<Vect>,
                         vcache:         &mut Vec<Vect>)
                         -> Vect {
    vcache.grow_set(nvpoints - 1, &na::zero(), na::zero());

    // FIXME: start with u or v, depending on which dimension has more control points.
    let vcache = vcache.as_mut_slice();

    for i in range(0, nvpoints) {
        let start = i * nupoints;
        let end   = start + nupoints;

        vcache[i] = bezier_curve::bezier_curve_at(control_points.slice(start, end), u, ucache);
    }

    bezier_curve::bezier_curve_at(vcache.slice(0, nvpoints), v, ucache)
}

/// Computes an infinite bounding cone of this surface.
#[cfg(not(dim4))]
pub fn bounding_cone_with_origin(points: &[Vect], origin: &Vect) -> (Vect, Scalar) {
    let mut axis  = points[0] - *origin;
    let mut theta = na::zero();

    // FIXME: this is not the best way to handle this (a better solution would be to loop on the
    // points until a non-zero vector is found).
    if axis.normalize().is_zero() {
        axis = na::zero();
        axis.set(0, na::one());
    }

    for pt in points.slice_from(1).iter() {
        let mut dir = *pt - *origin;

        if !dir.normalize().is_zero() {
            let alpha = na::dot(&dir, &axis).acos();

            if alpha > theta {
                // We are outside of the bounding cone: enlarge it to wrap both the old bounding
                // cone and the new point.

                let mut rot_axis = na::cross(&dir, &axis);

                if !rot_axis.normalize().is_zero() {
                    let dangle   = -(alpha - theta) * na::cast(0.5);
                    let rot      = na::append_rotation(&na::one::<RotationMatrix>(), &(rot_axis * dangle));

                    axis  = rot * axis;
                    theta = (alpha + theta) * na::cast(0.5);
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

/// Computes an infinite bounding cone of this surface.
#[cfg(dim4)] // we cannot use the previous algorithm because it requires a `na::cross`.
pub fn bounding_cone_with_origin(points: &[Vect], origin: &Vect) -> (Vect, Scalar) {
    // FIXME: this is a very coarse approximation
    let mut axis = bounding_volume::center(points) - *origin;

    if axis.normalize().is_zero() {
        axis = na::zero();
        axis.set(0, na::one());
    }

    let mut max_angle = na::zero();

    for pt in points.iter() {
        let mut dir = *pt - *origin;

        if !dir.normalize().is_zero() {
            let angle = na::dot(&dir, &axis).acos();

            if angle > max_angle {
                max_angle = angle;
            }
        }
    }

    (axis, max_angle)
}
