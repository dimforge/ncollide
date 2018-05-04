use std::f64;
#[cfg(feature = "dim3")]
use smallvec::SmallVec;

use na::{self, Real, Unit};
use math::{Isometry, Vector};

/// A convex cone with polyhedral faces and its apex at the origin.
///
/// A polyhedral cone is a set of half-lines forming a convex set. It
/// is usually used to bound a set of directions like normals and tangents.
/// It must be convex and can be generated from a finite set of vectors.
#[cfg(feature = "dim2")]
#[derive(Clone, Debug)]
pub enum PolyhedralCone<N: Real> {
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
    Span([Unit<Vector<N>>; 2]),
}

/// A convex cone with polyhedral faces and its apex at the origin.
///
/// A polyhedral cone is a set of half-lines forming a convex set. It
/// is usually used to bound a set of directions like normals and tangents.
/// It must be convex and can be generated from a finite set of vectors.
#[cfg(feature = "dim3")]
#[derive(Clone, Debug)]
pub enum PolyhedralCone<N: Real> {
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
    Span(SmallVec<[Unit<Vector<N>>; 4]>),
}

impl<N: Real> PolyhedralCone<N> {
    /// If this polyhedral cone spans a single half-line, returns its direction.
    pub fn unwrap_half_line(&self) -> Unit<Vector<N>> {
        if let PolyhedralCone::HalfLine(dir) = *self {
            dir
        } else {
            panic!("This polyhedral cone is not a half-line.")
        }
    }

    /// Applies the given transformation to each direction bounded by this cone.
    pub fn transform_by(&mut self, m: &Isometry<N>) {
        match *self {
            PolyhedralCone::HalfLine(ref mut dir) => *dir = m * *dir,
            PolyhedralCone::HalfSpace(ref mut normal) => *normal = m * *normal,
            PolyhedralCone::OrthogonalSubspace(ref mut normal) => *normal = m * *normal,
            PolyhedralCone::Span(ref mut generators) => for gen in generators {
                *gen = m * *gen;
            },
            PolyhedralCone::Full => {}
            PolyhedralCone::Empty => {}
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
    /// This test is much sheaper than `.contains()`.
    pub fn polar_contains_dir(&self, dir: &Unit<Vector<N>>) -> bool {
        let eps: N = na::convert(f64::consts::PI / 180.0 * 0.01);
        let c_eps = eps.cos();

        match *self {
            PolyhedralCone::Full => false,
            PolyhedralCone::Empty => true,
            PolyhedralCone::HalfLine(ref generator) => {
                na::dot(generator.as_ref(), dir.as_ref()) <= na::zero()
            }
            PolyhedralCone::HalfSpace(ref normal) => {
                na::dot(normal.as_ref(), dir.as_ref()) >= na::zero()
            }
            PolyhedralCone::OrthogonalSubspace(ref normal) => {
                na::dot(normal.as_ref(), dir.as_ref()).abs() >= N::one() - c_eps
            }
            PolyhedralCone::Span(ref generators) => {
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
            PolyhedralCone::Full => true,
            PolyhedralCone::Empty => false,
            PolyhedralCone::HalfLine(ref generator) => {
                na::dot(generator.as_ref(), dir.as_ref()) >= N::one() - c_eps
            }
            PolyhedralCone::HalfSpace(ref normal) => {
                na::dot(normal.as_ref(), dir.as_ref()) <= na::zero()
            }
            PolyhedralCone::OrthogonalSubspace(ref normal) => {
                na::dot(normal.as_ref(), dir.as_ref()).abs() <= c_eps
            }
            PolyhedralCone::Span(ref generators) => {
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
