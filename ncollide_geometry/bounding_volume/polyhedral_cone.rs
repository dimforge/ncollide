use smallvec::SmallVec;
use na::Unit;
use math::Vector;

#[derive(Clone, Debug)]
pub struct PolyhedralCone<V: Vector> {
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
    pub fn clear(&mut self) {
        self.generators.clear();
    }

    #[inline]
    pub fn add_generator(&mut self, gen: Unit<V>) {
        self.generators.push(gen);
    }
}
