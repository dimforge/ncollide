use nalgebra::na::Translation;
use nalgebra::na;
use math::{Scalar, Vect, RotationMatrix};
use bounding_volume::{BoundingVolume, BoundingSphere};

// FIXME: make a structure 'cone' ?
struct SpacializedCone {
    sphere:  BoundingSphere,
    axis:    Vect,
    angle:   Scalar,
    changle: Scalar,
    shangle: Scalar
}

impl SpacializedCone {
    pub fn new(sphere: BoundingSphere, axis: Vect, angle: Scalar) -> SpacializedCone {
        let axis = na::normalize(&axis);

        SpacializedCone {
            sphere:  sphere,
            axis:    axis,
            angle:   angle,
            changle: angle.cos(),
            shangle: angle.sin(),
        }
    }

    #[inline]
    pub fn sphere<'a>(&'a self) -> &'a BoundingSphere {
        &self.sphere
    }

    #[inline]
    pub fn axis<'a>(&'a self) -> &'a Vect {
        &self.axis
    }

    #[inline]
    pub fn hangle(&self) -> Scalar {
        self.hangle.clone()
    }
}

impl BoundingVolume for SpacializedCone {
    #[inline]
    fn intersects(&self, other: &SpacializedCone) -> bool {
        if self.sphere.intersects(&other.sphere) {
            let cangle = na::dot(&self.axis, &(-other.axis));

            // cos(a + b) = cos(b) cos(b) + sin(a) sin(b)
            let angsum = self.changle * other.changle - self.shangle * other.shangle;

            cangle >= angsum
        }
        else {
            false
        }
    }

    #[inline]
    fn contains(&self, other: &SpacializedCone) -> bool {
        if self.sphere.contains(&other.sphere) && self.changle >= other.changle {
            let cangle = na::dot(&self.axis, &other.axis);

            cangle >= self.changle
        }
        else {
            false
        }
    }

    #[inline]
    fn merge(&mut self, other: &SpacializedCone) {
        self.sphere.merge(other.sphere);

        // merge the cone
        let alpha = na::dot(&self.axis, &other.axis).acos();

        let mut rot_axis = na::cross(&self.axis, &other.axis);
        if !rot_axis.normalize().is_zero() {
            let dangle = (alpha - self.angle + other.angle) * na::cast(0.5f64);
            let rot    = na::append_rotation(&na::one::<RotationMatrix>(), &(rot_axis * dangle));

            self.axis    = rot * self.axis;
            self.angle   = na::clamp(self.angle + other.angle + alpha, na::zero(), Float::pi());
            self.changle = self.angle.cos();
            self.shangle = self.angle.sin();
        }
        else {
            self.angle   = Float::pi();
            self.changle = -na::one::<Scalar>();
            self.shangle = na::zero();
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
        self.sphere.center.clone()
    }

    #[inline]
    fn inv_translation(&self) -> Vect {
        -self.sphere.translation()
    }

    #[inline]
    fn append_translation(&mut self, dv: &Vect) {
        self.sphere.center = self.sphere.center + *dv
    }

    #[inline]
    fn append_translation_cpy(sc: &SpacializedCone, dv: &Vect) -> SpacializedCone {
        SpacializedCone::new(
            BoundingSphere::new(sc.sphere.center + *dv, sc.sphere.radius),
            sc.axis.clone(),
            sc.angle.clone())
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
        self.sphere.center = v
    }
}
