
use na::Real;

use math::Vector;

pub enum UpVector {
    X, Y, Z
}


#[derive(Clone, Debug)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Heightmap<N: Real> {
    heights: Vec<N>,
    scale: Vector<N>,
    up: UpVector

}

impl<N: Real> Heightmap<N> {
    pub fn new(heights: Vec<N>, scale: Vector<N>, up: UpVector) -> Self {
        Heightmap {
            heights, scale, up
        }
    }
}