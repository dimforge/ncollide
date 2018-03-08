use num::Zero;
use smallvec::SmallVec;

use na::{self, Unit};
use utils;
use math::Vector;

#[derive(Clone, Debug)]
pub struct PolyhedralCone<V: Vector> {
    // FIXME: specialization, 2D polycones cannot contain more than 2 generators.
    generators: SmallVec<[Unit<V>; 4]>,
}

impl<V: Vector> PolyhedralCone<V> {
    pub fn new() -> Self {
        Self::from_slice(&[])
    }

    pub fn from_slice(normals: &[Unit<V>]) -> Self {
        PolyhedralCone {
            generators: SmallVec::from_slice(normals),
        }
    }

    #[inline]
    pub fn len(&self) -> usize {
        self.generators.len()
    }

    #[inline]
    pub fn clear(&mut self) {
        self.generators.clear();
    }

    #[inline]
    pub fn add_generator(&mut self, gen: Unit<V>) {
        if na::dimension::<V>() == 2 && self.generators.len() == 2 {
            // FIXME: too restrictive?
            panic!("Polyhedral cone: 2D polyhedral cones cannot contain more than 2 generators.")
        }
        self.generators.push(gen);
    }

    pub fn project(&self, dir: &Unit<V>) -> Unit<V> {
        if self.generators.len() == 0 {
            *dir
        } else if self.generators.len() == 1 {
            self.generators[0]
        } else {
            if na::dimension::<V>() == 2 {
                assert!(self.generators.len() == 2);
                let perp1 = utils::perp2(&**dir, &*self.generators[0]);
                let perp2 = utils::perp2(&**dir, &*self.generators[1]);
                let _0 = V::Real::zero();

                match (perp1 > _0, perp2 > _0) {
                    (true, true) => self.generators[0],
                    (false, false) => self.generators[1],
                    (false, true) => *dir,
                    (true, false) => {
                        if perp1 <= -perp2 {
                            self.generators[0]
                        } else {
                            self.generators[1]
                        }
                    }
                }
            } else {
                unimplemented!()
            }
        }
    }
}
