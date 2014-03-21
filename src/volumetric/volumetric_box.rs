use nalgebra::na::Iterable;
use nalgebra::na;
use geom::Box;
use volumetric::Volumetric;
use math::{Scalar, Vect, AngularInertia};

#[cfg(dim2)]
use nalgebra::na::Indexable;

#[cfg(dim3)]
use nalgebra::na::Indexable;


/// Computes the volume of a box.
#[inline]
pub fn box_volume(half_extents: &Vect) -> Scalar {
    let mut res: Scalar = na::one();

    for half_extent in half_extents.iter() {
        res = res * *half_extent * na::cast(2.0)
    }

    res
}

#[cfg(dim2)]
impl Volumetric for Box {
    fn mass_properties(&self, density: &Scalar) -> (Scalar, Vect, AngularInertia) {
        let half_extents_w_margin = self.half_extents() + self.margin();
        let mass = box_volume(&half_extents_w_margin) * *density;

        let _2: Scalar   = na::cast(2.0);
        let _i12: Scalar = na::cast(1.0 / 12.0);
        let w       = _i12 * mass * _2 * _2;
        let ix      = w * half_extents_w_margin.at(0) * half_extents_w_margin.at(0);
        let iy      = w * half_extents_w_margin.at(1) * half_extents_w_margin.at(1);

        let mut res: AngularInertia = na::zero();

        res.set((0, 0), ix + iy);

        (mass, na::zero(), res)
    }
}

#[cfg(dim3)]
impl Volumetric for Box {
    fn mass_properties(&self, density: &Scalar) -> (Scalar, Vect, AngularInertia) {
        let half_extents_w_margin = self.half_extents() + self.margin();
        let mass = box_volume(&half_extents_w_margin) * *density;

        let _0: Scalar   = na::zero();
        let _2: Scalar   = na::cast(2.0);
        let _i12: Scalar = na::cast(1.0 / 12.0);
        let w       = _i12 * mass * _2 * _2;
        let ix      = w * half_extents_w_margin.at(0) * half_extents_w_margin.at(0);
        let iy      = w * half_extents_w_margin.at(1) * half_extents_w_margin.at(1);
        let iz      = w * half_extents_w_margin.at(2) * half_extents_w_margin.at(2);

        let mut res: AngularInertia = na::zero();

        res.set((0, 0), iy + iz);
        res.set((1, 1), ix + iz);
        res.set((2, 2), ix + iy);

        (mass, na::zero(), res)
    }
}

#[cfg(dim4)]
impl Volumetric for Box {
    fn mass_properties(&self, _: &Scalar) -> (Scalar, Vect, AngularInertia) {
        fail!("mass_properties is not yet implemented for 4d boxes.")
    }
}
