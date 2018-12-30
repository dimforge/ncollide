use arrayvec::ArrayVec;
use crate::bounding_volume::{self, CircularCone};
use crate::math::{Isometry, Vector};
use na::{self, Real, Unit};
use std::f64;

/// A convex cone with its apex at the origin.
///
/// A polyhedral cone is a set of half-lines forming a convex set. It
/// is usually used to bound a set of directions like normals and tangents.
/// It must be convex and can be generated from a finite set of vectors.
#[cfg(feature = "dim2")]
#[derive(Clone, Debug)]
pub enum ConicalApproximation<N: Real> {
    /// A polyhedral cone which is the whole space.
    Full,
    /// An empty cone containing only the zero vector.
    Empty,
    /// The half-line starting at the origin, pointing toward the given diretion.
    HalfLine(Unit<Vector<N>>),
    /// The half-space which boundary has the given diretion as normal.
    HalfSpace(Unit<Vector<N>>),
    /// The subspace orthogonal to the given diretion.
    OrthogonalSubspace(Unit<Vector<N>>),
    /// All the positive linear combinations of the given set of vectors.
    Span(ArrayVec<[Unit<Vector<N>>; 2]>),
    /// All the vectors that form an angle of at most `angle` with vhe vector `axis`.
    Circular {
        /// The circular conical approximation axis.
        axis: Unit<Vector<N>>,
        /// The circular conical approximation apex half-angle.
        angle: N,
    },
}

/// A convex cone with its apex at the origin.
///
/// A polyhedral cone is a set of half-lines forming a convex set. It
/// is usually used to bound a set of directions like normals and tangents.
/// It must be convex and can be generated from a finite set of vectors.
#[cfg(feature = "dim3")]
#[derive(Clone, Debug)]
pub enum ConicalApproximation<N: Real> {
    /// A polyhedral cone which is the whole space.
    Full,
    /// An empty cone containing only the zero vector.
    Empty,
    /// The half-line starting at the origin, pointing toward the given diretion.
    HalfLine(Unit<Vector<N>>),
    /// The half-space which boundary has the given direction as normal.
    HalfSpace(Unit<Vector<N>>),
    /// The subspace orthogonal to the given direction.
    OrthogonalSubspace(Unit<Vector<N>>),
    /// All the positive linear combinations of up to four vectors.
    Span(ArrayVec<[Unit<Vector<N>>; 4]>),
    /// All the vectors that form an angle of at most `angle` with vhe vector `axis`.
    Circular {
        /// The circular conical approximation axis.
        axis: Unit<Vector<N>>,
        /// The circular conical approximation apex half-angle.
        angle: N,
    },
}

impl<N: Real> ConicalApproximation<N> {
    /// If this polyhedral cone spans a single half-line, returns its direction.
    pub fn unwrap_half_line(&self) -> Unit<Vector<N>> {
        if let ConicalApproximation::HalfLine(dir) = *self {
            dir
        } else {
            panic!("This polyhedral cone is not a half-line.")
        }
    }

    /// Enlarge this conical approximation to contain the given direction.
    pub fn push(&mut self, dir: Unit<Vector<N>>) {
        // NOTE: there are a lot of return in there because we need to use `ccone` to avoid
        // borrowing issues when replacing a ::Span by a ::Circular.
        let ccone = match *self {
            ConicalApproximation::Full => {
                return;
            }
            ConicalApproximation::Empty => {
                *self = ConicalApproximation::HalfLine(dir);
                return;
            }
            ConicalApproximation::HalfLine(l) => {
                if !relative_eq!(l.dot(&dir), N::one()) {
                    let mut gens = ArrayVec::new();
                    gens.push(l);
                    gens.push(dir);
                    *self = ConicalApproximation::Span(gens)
                }
                return;
            }
            ConicalApproximation::HalfSpace(n) => {
                if n.dot(&dir) > N::zero() {
                    *self = ConicalApproximation::Full
                }
                return;
            }
            ConicalApproximation::OrthogonalSubspace(n) => {
                let dot = n.dot(&dir);
                if relative_eq!(dot, N::zero()) {
                    if dot < N::zero() {
                        *self = ConicalApproximation::HalfSpace(n)
                    } else {
                        *self = ConicalApproximation::HalfSpace(-n)
                    }
                }
                return;
            }
            ConicalApproximation::Span(ref mut generators) => {
                if generators.len() < 4 {
                    generators.push(dir);
                    return;
                } else {
                    CircularCone::from_vectors(generators.as_slice())
                }
            }
            ConicalApproximation::Circular { axis, angle } => {
                let mut res = CircularCone::Spread { axis, angle };
                res.push(dir);
                *self = res.into();
                return;
            }
        };

        *self = ccone.into();
    }

    /// Applies the given transformation to each direction bounded by this cone.
    pub fn transform_by(&mut self, m: &Isometry<N>) {
        match *self {
            ConicalApproximation::HalfLine(ref mut dir) => *dir = m * *dir,
            ConicalApproximation::HalfSpace(ref mut normal) => *normal = m * *normal,
            ConicalApproximation::OrthogonalSubspace(ref mut normal) => *normal = m * *normal,
            ConicalApproximation::Span(ref mut generators) => {
                for gen in generators {
                    *gen = m * *gen;
                }
            }
            ConicalApproximation::Circular { ref mut axis, .. } => *axis = m * &*axis,
            ConicalApproximation::Full => {}
            ConicalApproximation::Empty => {}
        }
    }

    /// Tests whether the given vector is contained by this cone.
    pub fn contains(&self, v: &Vector<N>) -> bool {
        if let Some(dir) = Unit::try_new(*v, N::default_epsilon()) {
            self.contains_dir(&dir)
        } else {
            true
        }
    }

    /// Tests if the polar of this cone contains the given direction.
    ///
    /// This test is much cheaper than `.contains()`.
    pub fn polar_contains_dir(&self, dir: &Unit<Vector<N>>) -> bool {
        let eps: N = na::convert(f64::consts::PI / 180.0 * 0.01);
        let c_eps = eps.cos();

        match *self {
            ConicalApproximation::Full => false,
            ConicalApproximation::Empty => true,
            ConicalApproximation::HalfLine(ref generator) => {
                na::dot(generator.as_ref(), dir.as_ref()) <= na::zero()
            }
            ConicalApproximation::HalfSpace(ref normal) => {
                na::dot(normal.as_ref(), dir.as_ref()) >= na::zero()
            }
            ConicalApproximation::OrthogonalSubspace(ref normal) => {
                na::dot(normal.as_ref(), dir.as_ref()).abs() >= N::one() - c_eps
            }
            ConicalApproximation::Circular { ref axis, angle } => {
                bounding_volume::circular_cone::cone_polar_contains_dir(axis, angle, dir)
            }
            ConicalApproximation::Span(ref generators) => {
                for g in generators {
                    if na::dot(g.as_ref(), dir.as_ref()) > na::zero() {
                        return false;
                    }
                }
                true
            }
        }
    }

    /// Tests if this cone contains the given unit direction.
    pub fn contains_dir(&self, dir: &Unit<Vector<N>>) -> bool {
        let eps: N = na::convert(f64::consts::PI / 180.0 * 0.01);
        let c_eps = eps.cos();

        match *self {
            ConicalApproximation::Full => true,
            ConicalApproximation::Empty => false,
            ConicalApproximation::HalfLine(ref generator) => {
                na::dot(generator.as_ref(), dir.as_ref()) >= N::one() - c_eps
            }
            ConicalApproximation::HalfSpace(ref normal) => {
                na::dot(normal.as_ref(), dir.as_ref()) <= na::zero()
            }
            ConicalApproximation::OrthogonalSubspace(ref normal) => {
                na::dot(normal.as_ref(), dir.as_ref()).abs() <= c_eps
            }
            ConicalApproximation::Circular { ref axis, angle } => {
                bounding_volume::circular_cone::cone_contains_dir(axis, angle, dir)
            }
            ConicalApproximation::Span(ref generators) => {
                #[cfg(feature = "dim2")]
                {
                    // NOTE: the following assumes the polycone
                    // generator are ordered in CCW order.
                    let perp1 = dir.as_ref().perp(&*generators[0]);
                    let perp2 = dir.as_ref().perp(&*generators[1]);

                    perp1 <= N::zero() && perp2 >= N::zero()
                }

                #[cfg(feature = "dim3")]
                {
                    // NOTE: the following does not makes any assumptions on the
                    // polycone orientation.
                    if generators.len() == 1 {
                        // The polycone is degenerate and actually has only one generactor.
                        let eps: N = na::convert(f64::consts::PI / 180.0 * 0.1);
                        let c_eps = eps.cos();
                        let dot = na::dot(&*generators[0], dir.as_ref());
                        dot >= c_eps
                    } else if generators.len() == 2 {
                        let eps = na::convert(f64::consts::PI / 180.0 * 0.1);
                        let normal = generators[1].cross(&*generators[0]);

                        if let Some(normal) = Unit::try_new(normal, na::zero()) {
                            if na::dot(&*normal, dir.as_ref()).abs() > eps {
                                return false;
                            }

                            let middle =
                                (*generators[0] + *generators[1]) * na::convert::<_, N>(0.5);
                            if na::dot(&middle, dir.as_ref()) < na::zero() {
                                return false;
                            }

                            let cross1 = generators[0].cross(dir.as_ref());
                            let cross2 = generators[1].cross(dir.as_ref());

                            na::dot(&cross1, &*normal) * na::dot(&cross2, &*normal) <= na::zero()
                        } else {
                            // FIXME: duplicate code with the case where we only have one generator.
                            // The polycone is degenerate and actually has only one generactor.
                            let c_eps = eps.cos();
                            let dot = na::dot(&*generators[0], dir.as_ref());
                            dot >= c_eps
                        }
                    } else {
                        let mut sign = N::zero();
                        let mut center = Vector::zeros();

                        for i1 in 0..generators.len() {
                            let i2 = (i1 + 1) % generators.len();
                            let cross = generators[i1].cross(generators[i2].as_ref());
                            let dot = na::dot(dir.as_ref(), &cross);
                            center += generators[i1].unwrap();

                            if sign.is_zero() {
                                sign = dot
                            } else if sign * dot < na::zero() {
                                return false;
                            }
                        }

                        // FIXME: is this a sufficient condition to determine if the
                        // dir is not of the opposite cone?
                        na::dot(&center, dir.as_ref()) >= na::zero()
                    }
                }
            }
        }
    }
}

impl<N: Real> From<CircularCone<N>> for ConicalApproximation<N> {
    fn from(cone: CircularCone<N>) -> Self {
        match cone {
            CircularCone::Full => ConicalApproximation::Full,
            CircularCone::Empty => ConicalApproximation::Empty,
            CircularCone::Spread { axis, angle } => ConicalApproximation::Circular { axis, angle },
        }
    }
}
