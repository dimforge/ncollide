use bounding_volume::BoundingVolume;
use math::{Point, Vector};
use na::{self, Real, Unit};

/// A cone with a circular basis and its apex at the origin.
///
/// A circular cone is a set of half-lines emanating from its apex and forming an angle of at most `angle` with its `axis`.
/// It is usually used to bound a set of directions like normals and tangents.
/// It is convex and have a circular basis.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum CircularCone<N: Real> {
    /// A cone which is the whole space.
    Full,
    /// An empty cone containing only the zero vector.
    Empty,
    /// All the vectors emanating from the origin, with a maximal `angle` wrt the given `axis`.
    Spread {
        /// The cone axis.
        axis: Unit<Vector<N>>,
        /// Half of the cone apex angle, i.e., the largest angle possible between the axis and a vector contained by this cone.
        angle: N,
    },
}

// FIXME: rewrite all those without calls to acos()
// (by performing tests on the cos themselves instead of the actual angles).

impl<N: Real> CircularCone<N> {
    /// Creates a circular cone from a set of vectors.
    pub fn from_vectors(dirs: &[Unit<Vector<N>>]) -> Self {
        let mut res = CircularCone::Empty;

        for dir in dirs {
            res.push(*dir)
        }

        res
    }

    /// Returns `true` if this cone is empty.
    pub fn is_empty(&self) -> bool {
        *self == CircularCone::Empty
    }

    /// Enlarge this cone so it contains `dir` too.
    pub fn push(&mut self, dir: Unit<Vector<N>>) {
        match *self {
            CircularCone::Full => {}
            CircularCone::Empty => {
                *self = CircularCone::Spread {
                    axis: dir,
                    angle: N::zero(),
                }
            }
            CircularCone::Spread {
                ref mut axis,
                ref mut angle,
            } => {
                let dot = axis.dot(&dir);
                let delta_ang = dot.acos();

                if delta_ang <= *angle {
                    // The current cone already contains dir.
                } else {
                    let ortho = *dir - **axis * dot;
                    if let Some(basis2) = Unit::try_new(ortho, N::zero()) {
                        let hang = delta_ang * na::convert(0.5);
                        let (s, c) = hang.sin_cos();
                        *axis = Unit::new_unchecked(**axis * c + *basis2 * s);
                        *angle = hang + *angle * na::convert(0.5);
                    }
                    // Otherwise, dir and axis are collinear so there is nothing more to do.
                }
            }
        }
    }

    /// Returns `true` if this cone intersects `other`.
    pub fn intersects(&self, other: &Self) -> bool {
        match (self, other) {
            (CircularCone::Empty, _) => false,
            (_, CircularCone::Empty) => false,
            (CircularCone::Full, _) => true,
            (_, CircularCone::Full) => true,
            (
                CircularCone::Spread {
                    axis: axis1,
                    angle: angle1,
                },
                CircularCone::Spread {
                    axis: axis2,
                    angle: angle2,
                },
            ) => {
                let ang = axis1.dot(&axis2).acos();
                ang <= *angle1 + *angle2
            }
        }
    }

    /// Tests if this circular cone, extended to be a double cone, intersects the `other` circular cone, also seen as a double cone.
    pub fn double_cones_intersect(&self, other: &Self) -> bool {
        match (self, other) {
            (CircularCone::Empty, _) => false,
            (_, CircularCone::Empty) => false,
            (CircularCone::Full, _) => true,
            (_, CircularCone::Full) => true,
            (
                CircularCone::Spread {
                    axis: axis1,
                    angle: angle1,
                },
                CircularCone::Spread {
                    axis: axis2,
                    angle: angle2,
                },
            ) => {
                let ang = axis1.dot(&axis2).acos();
                ang <= *angle1 + *angle2 || (N::pi() - ang) <= *angle1 + *angle2
            }
        }
    }

    /// Returns `true` if this cone contains `other`.
    pub fn contains(&self, other: &Self) -> bool {
        match (self, other) {
            (CircularCone::Empty, _) => false,
            (CircularCone::Full, _) => *other != CircularCone::Empty,
            (_, CircularCone::Full) => false,
            (_, CircularCone::Empty) => true,
            (
                CircularCone::Spread {
                    axis: axis1,
                    angle: angle1,
                },
                CircularCone::Spread {
                    axis: axis2,
                    angle: angle2,
                },
            ) => {
                let ang = axis1.dot(&axis2).acos();
                ang + *angle2 <= *angle1
            }
        }
    }

    /// Merges this cone with `other` in-place.
    pub fn merge(&mut self, other: &Self) {
        *self = self.merged(other)
    }

    /// Merges this cone with `other`.
    pub fn merged(&self, other: &Self) -> Self {
        match (self, other) {
            (CircularCone::Empty, _) => *other,
            (CircularCone::Full, _) => CircularCone::Full,
            (_, CircularCone::Empty) => *self,
            (_, CircularCone::Full) => CircularCone::Full,
            (
                CircularCone::Spread {
                    axis: axis1,
                    angle: angle1,
                },
                CircularCone::Spread {
                    axis: axis2,
                    angle: angle2,
                },
            ) => {
                let dot = axis1.dot(&axis2);
                let ang = dot.acos();
                if ang + *angle1 <= *angle2 {
                    // self is contained in other
                    // so there is nothing to do for the merge.
                    *self
                } else if ang + *angle2 <= *angle1 {
                    // other is contained in self
                    *other
                } else {
                    let ortho = **axis2 - **axis1 * dot;

                    if let Some(basis2) = Unit::try_new(ortho, N::zero()) {
                        let partial_sum = (ang + *angle2) * na::convert(0.5);
                        let (s, c) = partial_sum.sin_cos();
                        let new_axis = **axis1 * c + *basis2 * s;
                        CircularCone::Spread {
                            axis: Unit::new_unchecked(new_axis),
                            angle: partial_sum + *angle1 * na::convert(0.5),
                        }
                    } else {
                        // This should be unreachable because that means both axii are superimposed so one
                        // of the first `if` statements above should have kicked in already.
                        // But this might happen due to rounding errors. Just return the
                        // cone with the largest angle.
                        if *angle2 > *angle1 {
                            *other
                        } else {
                            *self
                        }
                    }
                }
            }
        }
    }
}

/// Checks if the unit vector `dir` is inside of the circular cone described by the given `axis` and apex half-angle `angle`.
pub fn cone_contains_dir<N: Real>(axis: &Unit<Vector<N>>, angle: N, dir: &Unit<Vector<N>>) -> bool {
    let ang = axis.dot(dir).acos();
    ang <= angle
}

/// Checks if the unit vector `dir` is inside of the polar of the circular cone described by the given `axis` and apex half-angle `angle`.
pub fn cone_polar_contains_dir<N: Real>(
    axis: &Unit<Vector<N>>,
    angle: N,
    dir: &Unit<Vector<N>>,
) -> bool {
    let ang = axis.dot(dir).acos();
    ang >= angle + N::frac_pi_2()
}
