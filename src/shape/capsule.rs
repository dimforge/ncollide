//! Support mapping based Capsule shape.

use na::{self, RealField, Unit};

use crate::math::{Point, Vector};
use crate::query::{Contact, ContactKinematic, ContactPreprocessor};
use crate::shape::{FeatureId, Segment, SupportMap};

/// SupportMap description of a capsule shape with its principal axis aligned with the `y` axis.
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(PartialEq, Debug, Copy, Clone)]
pub struct Capsule<N> {
    /// The half-height of the capsule's cylindrical part.
    pub half_height: N,
    /// The radius of the capsule.
    pub radius: N,
}

impl<N: RealField> Capsule<N> {
    /// Creates a new capsule.
    ///
    /// # Arguments:
    /// * `half_height` - the half length of the capsule along the `y` axis.
    /// * `radius` - radius of the rounded part of the capsule.
    pub fn new(half_height: N, radius: N) -> Capsule<N> {
        Capsule {
            half_height,
            radius,
        }
    }

    /// The capsule half length along its local `y` axis.
    #[inline]
    #[deprecated(note = "use the `self.half_height` public field directly.")]
    pub fn half_height(&self) -> N {
        self.half_height
    }

    /// The capsule height along its local `y` axis.
    #[inline]
    pub fn height(&self) -> N {
        self.half_height * na::convert(2.0)
    }

    /// The radius of the capsule's rounded part.
    #[inline]
    #[deprecated(note = "use the `self.radius` public field directly.")]
    pub fn radius(&self) -> N {
        self.radius
    }

    /// The segment that, once dilated by `self.radius` yields this capsule.
    #[inline]
    pub fn segment(&self) -> Segment<N> {
        let mut a = Point::origin();
        let mut b = Point::origin();
        a.y = -self.half_height;
        b.y = self.half_height;

        Segment::new(a, b)
    }

    /// The contact preprocessor to be used for contact determination with this capsule.
    #[inline]
    pub fn contact_preprocessor(&self) -> impl ContactPreprocessor<N> {
        CapsuleContactPreprocessor {
            radius: self.radius,
        }
    }
}

impl<N: RealField> SupportMap<N> for Capsule<N> {
    #[inline]
    fn local_support_point(&self, dir: &Vector<N>) -> Point<N> {
        self.local_support_point_toward(&Unit::new_normalize(*dir))
    }

    #[inline]
    fn local_support_point_toward(&self, dir: &Unit<Vector<N>>) -> Point<N> {
        let mut res: Vector<N> = na::zero();
        res[1] = self.half_height.copysign(dir[1]);
        Point::from(res + **dir * self.radius)
    }
}

struct CapsuleContactPreprocessor<N: RealField> {
    radius: N,
}

impl<N: RealField> CapsuleContactPreprocessor<N> {
    //    pub fn new(radius: N) -> Self {
    //        CapsuleContactPreprocessor {
    //            radius
    //        }
    //    }
}

impl<N: RealField> ContactPreprocessor<N> for CapsuleContactPreprocessor<N> {
    fn process_contact(
        &self,
        c: &mut Contact<N>,
        kinematic: &mut ContactKinematic<N>,
        is_first: bool,
    ) -> bool {
        // Fix the feature ID.
        let feature = if is_first {
            kinematic.feature1()
        } else {
            kinematic.feature2()
        };

        let actual_feature = match feature {
            FeatureId::Vertex(i) => FeatureId::Face(i),
            #[cfg(feature = "dim3")]
            FeatureId::Edge(_) => FeatureId::Face(2),
            FeatureId::Face(i) => FeatureId::Face(2 + i),
            FeatureId::Unknown => return false,
        };

        if is_first {
            kinematic.set_feature1(actual_feature);
            kinematic.set_dilation1(self.radius);
            c.world1 += *c.normal * self.radius;
            c.depth += self.radius;
        } else {
            kinematic.set_feature2(actual_feature);
            kinematic.set_dilation2(self.radius);
            c.world2 -= *c.normal * self.radius;
            c.depth += self.radius;
        }

        true
    }
}
