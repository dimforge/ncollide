use std::num::Zero;
use nalgebra::na::{Norm, Translation};
use nalgebra::na;
use math::{Scalar, Vect, RotationMatrix};
use bounding_volume::{BoundingVolume, BoundingSphere};

// FIXME: make a structure 'cone' ?
#[deriving(Show, PartialEq, Clone, Encodable, Decodable)]
/// A normal cone with a bounding sphere.
pub struct SpacializedCone {
    sphere:  BoundingSphere,
    axis:    Vect,
    hangle:  Scalar,
}

impl SpacializedCone {
    /// Creates a new spacialized cone with a given bounding sphere, axis, and half-angle.
    pub fn new(sphere: BoundingSphere, axis: Vect, hangle: Scalar) -> SpacializedCone {
        let axis = na::normalize(&axis);

        SpacializedCone {
            sphere:  sphere,
            axis:    axis,
            hangle:   hangle
        }
    }

    /// The bounding sphere of this spacialized cone.
    #[inline]
    pub fn sphere<'a>(&'a self) -> &'a BoundingSphere {
        &self.sphere
    }

    /// This cone axis.
    #[inline]
    pub fn axis<'a>(&'a self) -> &'a Vect {
        &self.axis
    }

    /// This cone half angle.
    #[inline]
    pub fn hangle(&self) -> Scalar {
        self.hangle.clone()
    }
}

#[not_dim4]
impl BoundingVolume for SpacializedCone {
    #[inline]
    fn intersects(&self, other: &SpacializedCone) -> bool {
        if self.sphere.intersects(&other.sphere) {
            let dangle = na::dot(&self.axis, &(-other.axis));
            println!("dot: {}", dangle);
            let dangle = na::clamp(dangle, -na::one::<Scalar>(), na::one()).acos();
            let angsum = self.hangle + other.hangle;

            println!("c: {} a: {}", dangle, angsum);

            dangle <= angsum
        }
        else {
            false
        }
    }

    #[inline]
    fn contains(&self, other: &SpacializedCone) -> bool {
        if self.sphere.contains(&other.sphere) {
            fail!("Not yet implemented.")
        }
        else {
            false
        }
    }

    #[inline]
    fn merge(&mut self, other: &SpacializedCone) {
        self.sphere.merge(&other.sphere);

        // merge the cone
        let alpha = na::clamp(na::dot(&self.axis, &other.axis), -na::one::<Scalar>(), na::one()).acos();

        let mut rot_axis = na::cross(&self.axis, &other.axis);
        if !rot_axis.normalize().is_zero() {
            let dangle = (alpha - self.hangle + other.hangle) * na::cast(0.5f64);
            let rot    = na::append_rotation(&na::one::<RotationMatrix>(), &(rot_axis * dangle));

            self.axis    = rot * self.axis;
            self.hangle  = na::clamp(self.hangle + other.hangle + alpha, na::zero(), Float::pi());

            // println!("merged {} {} with hangle: {}", *self, *other, self.hangle);
        }
        else {
            // This happens if alpha ~= 0 or alpha ~= pi.
            if alpha > na::one() { // NOTE: 1.0 is just a randomly chosen number in-between 0 and pi.
                // alpha ~= pi
                self.hangle = alpha;
            }
            else {
                // alpha ~= 0, do nothing.
            }
        }
    }

    #[inline]
    fn merged(&self, other: &SpacializedCone) -> SpacializedCone {
        let mut res = self.clone();

        res.merge(other);

        res
    }
}

impl Translation<Vect> for SpacializedCone {
    #[inline]
    fn translation(&self) -> Vect {
        self.sphere.center().clone()
    }

    #[inline]
    fn inv_translation(&self) -> Vect {
        -self.sphere.translation()
    }

    #[inline]
    fn append_translation(&mut self, dv: &Vect) {
        self.sphere.append_translation(dv);
    }

    #[inline]
    fn append_translation_cpy(sc: &SpacializedCone, dv: &Vect) -> SpacializedCone {
        SpacializedCone::new(
            Translation::append_translation_cpy(&sc.sphere, dv),
            sc.axis.clone(),
            sc.hangle.clone())
    }

    #[inline]
    fn prepend_translation(&mut self, dv: &Vect) {
        self.sphere.append_translation(dv)
    }

    #[inline]
    fn prepend_translation_cpy(sc: &SpacializedCone, dv: &Vect) -> SpacializedCone {
        Translation::append_translation_cpy(sc, dv)
    }

    #[inline]
    fn set_translation(&mut self, v: Vect) {
        self.sphere.set_translation(v)
    }
}
