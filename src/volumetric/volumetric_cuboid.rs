use nalgebra::na::Indexable;
use nalgebra::na::Iterable;
use nalgebra::na;
use geom::Cuboid;
use volumetric::Volumetric;
use math::{Scalar, Vect, AngularInertia};


/// Computes the volume of a cuboid.
#[inline]
pub fn cuboid_volume(half_extents: &Vect) -> Scalar {
    let mut res: Scalar = na::one();

    for half_extent in half_extents.iter() {
        res = res * *half_extent * na::cast(2.0f64)
    }

    res
}

#[dim2]
impl Volumetric for Cuboid {
    #[inline]
    fn volume(&self) -> Scalar {
        let half_extents_w_margin = self.half_extents() + self.margin();

        cuboid_volume(&half_extents_w_margin)
    }

    #[inline]
    fn center_of_mass(&self) -> Vect {
        na::zero()
    }

    #[inline]
    fn unit_angular_inertia(&self) -> AngularInertia {
        let half_extents_w_margin = self.half_extents() + self.margin();

        let _2: Scalar   = na::cast(2.0f64);
        let _i12: Scalar = na::cast(1.0f64 / 12.0);
        let w       = _i12 * _2 * _2;
        let ix      = w * half_extents_w_margin.at(0) * half_extents_w_margin.at(0);
        let iy      = w * half_extents_w_margin.at(1) * half_extents_w_margin.at(1);

        let mut res: AngularInertia = na::zero();

        res.set((0, 0), ix + iy);

        res
    }
}

#[dim3]
impl Volumetric for Cuboid {
    #[inline]
    fn volume(&self) -> Scalar {
        let half_extents_w_margin = self.half_extents() + self.margin();

        cuboid_volume(&half_extents_w_margin)
    }

    #[inline]
    fn center_of_mass(&self) -> Vect {
        na::zero()
    }

    #[inline]
    fn unit_angular_inertia(&self) -> AngularInertia {
        let half_extents_w_margin = self.half_extents() + self.margin();

        let _0: Scalar   = na::zero();
        let _2: Scalar   = na::cast(2.0f64);
        let _i12: Scalar = na::cast(1.0f64 / 12.0);
        let w       = _i12 * _2 * _2;
        let ix      = w * half_extents_w_margin.at(0) * half_extents_w_margin.at(0);
        let iy      = w * half_extents_w_margin.at(1) * half_extents_w_margin.at(1);
        let iz      = w * half_extents_w_margin.at(2) * half_extents_w_margin.at(2);

        let mut res: AngularInertia = na::zero();

        res.set((0, 0), iy + iz);
        res.set((1, 1), ix + iz);
        res.set((2, 2), ix + iy);

        res
    }
}
