//! Axis Aligned Bounding Box.

use na::{Translation, POrd, Translate, Bounded};
use na;
use bounding_volume::BoundingVolume;
use math::{Scalar, Point, Vect};

/// Trait of objects that can be bounded by an AABB.
pub trait HasAABB<P, M> {
    /// The object’s AABB.
    fn aabb(&self, &M) -> AABB<P>;
}

/// An Axis Aligned Bounding Box.
#[deriving(Show, PartialEq, Clone, Encodable, Decodable)]
pub struct AABB<P> {
    mins: P,
    maxs: P
}

impl<P: POrd> AABB<P> {
    /// Creates a new AABB.
    ///
    /// # Arguments:
    ///   * `mins` - position of the point with the smallest coordinates.
    ///   * `maxs` - position of the point with the highest coordinates. Each component of `mins`
    ///   must be smaller than the related components of `maxs`.
    pub fn new(mins: P, maxs: P) -> AABB<P> {
        assert!(na::partial_le(&mins, &maxs));

        AABB {
            mins: mins,
            maxs: maxs
        }
    }
}

impl<P: Neg<P> + POrd + Bounded> AABB<P> {
    /// Creates an invalid AABB with:
    /// * `mins = Bounded::max_value()`
    /// * `maxs = Bounded::max_value()`.
    /// This is useful to build aabb using merges.
    pub fn new_invalid() -> AABB<P> {
        let _max: P = Bounded::max_value();
        AABB {
            mins: Bounded::max_value(),
            maxs: -_max,
        }
    }
}

impl<P> AABB<P> {
    /// Reference to the AABB point with the smallest components along each axis.
    #[inline]
    pub fn mins(&self) -> &P {
        &self.mins
    }

    /// Reference to the AABB point with the biggest components along each axis.
    #[inline]
    pub fn maxs(&self) -> &P {
        &self.maxs
    }
}

impl<N, P, V> AABB<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    /// The center of this AABB.
    #[inline]
    pub fn center(&self) -> P {
        na::center(&self.mins, &self.maxs)
    }

    /// The half extents of this AABB.
    #[inline]
    pub fn half_extents(&self) -> V {
        (self.maxs - self.mins) / na::cast(2.0f64)
    }
}

impl<N, P, V> BoundingVolume<N> for AABB<P>
    where N: Scalar,
          P: Point<N, V> {
    #[inline]
    fn intersects(&self, other: &AABB<P>) -> bool {
        na::partial_le(&self.mins, &other.maxs) &&
        na::partial_ge(&self.maxs, &other.mins)
    }

    #[inline]
    fn contains(&self, other: &AABB<P>) -> bool {
        na::partial_le(&self.mins, &other.mins) &&
        na::partial_ge(&self.maxs, &other.maxs)
    }

    #[inline]
    fn merge(&mut self, other: &AABB<P>) {
        self.mins = na::inf(&self.mins, &other.mins);
        self.maxs = na::sup(&self.maxs, &other.maxs);
    }

    #[inline]
    fn merged(&self, other: &AABB<P>) -> AABB<P> {
        AABB {
            mins: na::inf(&self.mins, &other.mins),
            maxs: na::sup(&self.maxs, &other.maxs)
        }
    }

    #[inline]
    fn loosen(&mut self, amount: N) {
        assert!(amount >= na::zero(), "The loosening margin must be positive.");
        self.mins = self.mins.sub_s(&amount);
        self.maxs = self.maxs.add_s(&amount);
    }

    #[inline]
    fn loosened(&self, amount: N) -> AABB<P> {
        assert!(amount >= na::zero(), "The loosening margin must be positive.");
        AABB {
            mins: self.mins.sub_s(&amount),
            maxs: self.maxs.add_s(&amount)
        }
    }

    #[inline]
    fn tighten(&mut self, amount: N) {
        assert!(amount >= na::zero(), "The tightening margin must be positive.");
        self.mins = self.mins.add_s(&amount);
        self.maxs = self.maxs.sub_s(&amount);
        assert!(na::partial_le(&self.mins, &self.maxs), "The tightening margin is to large.");
    }

    #[inline]
    fn tightened(&self, amount: N) -> AABB<P> {
        assert!(amount >= na::zero(), "The tightening margin must be positive.");

        AABB::new(self.mins.add_s(&amount), self.maxs.sub_s(&amount))
    }
}

impl<N, P, V> Translation<V> for AABB<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> + Translate<P> {
    #[inline]
    fn translation(&self) -> V {
        na::center(&self.mins, &self.maxs).to_vec()
    }

    #[inline]
    fn inv_translation(&self) -> V {
        -self.translation()
    }

    #[inline]
    fn append_translation(&mut self, dv: &V) {
        self.mins = self.mins + *dv;
        self.maxs = self.maxs + *dv;
    }

    #[inline]
    fn append_translation_cpy(aabb: &AABB<P>, dv: &V) -> AABB<P> {
        AABB::new(aabb.mins + *dv, aabb.maxs + *dv)
    }

    #[inline]
    fn prepend_translation(&mut self, dv: &V) {
        self.append_translation(dv)
    }

    #[inline]
    fn prepend_translation_cpy(aabb: &AABB<P>, dv: &V) -> AABB<P> {
        Translation::append_translation_cpy(aabb, dv)
    }

    #[inline]
    fn set_translation(&mut self, v: V) {
        let center = self.translation();
        let total_translation = center + v;

        self.mins = na::inv_translate(&total_translation, &self.mins);
        self.maxs = na::inv_translate(&total_translation, &self.maxs);
    }
}
