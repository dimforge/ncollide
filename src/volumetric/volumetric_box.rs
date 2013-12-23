use std::num::{Zero, One};
use nalgebra::na::{Cast, VecExt, Indexable};
use nalgebra::na;
use geom::Box;
use volumetric::Volumetric;


#[inline]
pub fn box_volume<N:  Zero + One + Cast<f32> + Num + Clone,
                  V:  Clone + VecExt<N>>(
                  half_extents: &V)
                  -> N {
    let mut res: N = na::one();

    for half_extent in half_extents.iter() {
        res = res * *half_extent * Cast::from(2.0)
    }

    res
}

impl<N:  Zero + One + Cast<f32> + Num + Clone,
     V:  Clone + VecExt<N>,
     II: Zero + Indexable<(uint, uint), N>>
Volumetric<N, V, II> for Box<N, V> {
    fn mass_properties(&self, density: &N) -> (N, V, II) {
        let half_extents_w_margin = self.half_extents().add_s(&self.margin());
        let mass = box_volume(&half_extents_w_margin) * *density;
        let dim  = na::dim::<V>();

        if dim == 2 {
            let _2: N   = Cast::from(2.0);
            let _i12: N = Cast::from(1.0 / 12.0);
            let w       = _i12 * mass * _2 * _2;
            let ix      = w * half_extents_w_margin.at(0) * half_extents_w_margin.at(0);
            let iy      = w * half_extents_w_margin.at(1) * half_extents_w_margin.at(1);

            let mut res: II = na::zero();

            res.set((0, 0), ix + iy);

            (mass, na::zero(), res)
        }
        else if dim == 3 {
            let _0: N   = na::zero();
            let _2: N   = Cast::from(2.0);
            let _i12: N = Cast::from(1.0 / 12.0);
            let w       = _i12 * mass * _2 * _2;
            let ix      = w * half_extents_w_margin.at(0) * half_extents_w_margin.at(0);
            let iy      = w * half_extents_w_margin.at(1) * half_extents_w_margin.at(1);
            let iz      = w * half_extents_w_margin.at(2) * half_extents_w_margin.at(2);

            let mut res: II = na::zero();

            res.set((0, 0), iy + iz);
            res.set((1, 1), ix + iz);
            res.set((2, 2), ix + iy);

            (mass, na::zero(), res)
        }
        else {
            fail!("Inertia tensor for n-dimensional boxes, n > 3, is not implemented.")
        }
    }
}

