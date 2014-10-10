use na::{Iterable, Indexable};
use na;
use geom::Cuboid;
use volumetric::Volumetric;
use math::{Scalar, Point, Vect, AngularInertia};


/// Computes the volume of a cuboid.
#[inline]
pub fn cuboid_volume(half_extents: &Vect) -> Scalar {
    let mut res: Scalar = na::one();

    for half_extent in half_extents.iter() {
        res = res * *half_extent * na::cast(2.0f64)
    }

    res
}

#[cfg(feature = "2d")]
impl Volumetric for Cuboid {
    #[inline]
    fn surface(&self) -> Scalar {
        (self.half_extents().x + self.half_extents().y) * na::cast(4.0f64)
    }

    #[inline]
    fn volume(&self) -> Scalar {
        cuboid_volume(&self.half_extents())
    }

    #[inline]
    fn center_of_mass(&self) -> Point {
        na::orig()
    }

    #[inline]
    fn unit_angular_inertia(&self) -> AngularInertia {
        let _2: Scalar   = na::cast(2.0f64);
        let _i12: Scalar = na::cast(1.0f64 / 12.0);
        let w       = _i12 * _2 * _2;
        let ix      = w * self.half_extents().at(0) * self.half_extents().at(0);
        let iy      = w * self.half_extents().at(1) * self.half_extents().at(1);

        let mut res: AngularInertia = na::zero();

        res.set((0, 0), ix + iy);

        res
    }
}

#[cfg(feature = "3d")]
impl Volumetric for Cuboid {
    #[inline]
    fn surface(&self) -> Scalar {
        let he = self.half_extents();
        let xx = he.x + he.x;
        let yy = he.y + he.y;
        let zz = he.z + he.z;

        let side_xy = xx * yy;
        let side_xz = xx * zz;
        let side_yz = yy * zz;

        (side_xy + side_xz + side_yz) * na::cast(2.0f64)
    }

    #[inline]
    fn volume(&self) -> Scalar {
        cuboid_volume(&self.half_extents())
    }

    #[inline]
    fn center_of_mass(&self) -> Point {
        na::orig()
    }

    #[inline]
    fn unit_angular_inertia(&self) -> AngularInertia {
        let _0: Scalar   = na::zero();
        let _2: Scalar   = na::cast(2.0f64);
        let _i12: Scalar = na::cast(1.0f64 / 12.0);
        let w       = _i12 * _2 * _2;
        let ix      = w * self.half_extents().at(0) * self.half_extents().at(0);
        let iy      = w * self.half_extents().at(1) * self.half_extents().at(1);
        let iz      = w * self.half_extents().at(2) * self.half_extents().at(2);

        let mut res: AngularInertia = na::zero();

        res.set((0, 0), iy + iz);
        res.set((1, 1), ix + iz);
        res.set((2, 2), ix + iy);

        res
    }
}
