use narrow::{CollisionDetector, Contact};

/// A collision detector that does nothing.
#[deriving(Encodable, Decodable)]
pub struct Empty<P, V, M, G1, G2> {
    dummy: uint // FIXME: useless, but zero-sized structure ICE when used cross-crate.
}

impl<N, P, V, M, G1, G2> Empty<P, V, M, G1, G2> {
    /// Creates a new empty collision detector.
    pub fn new() -> Empty<P, V, M, G1, G2> {
        Empty {
            dummy: 0
        }
    }
}

impl<N, P, V, M, G1, G2> CollisionDetector<N, P, V, M, G1, G2> for Empty<P, V, M, G1, G2> {
    fn update(&mut self, _: &M, _: &G1, _: &M, _: &G2) {
    }

    fn num_colls(&self) -> uint {
        0
    }

    fn colls(&self, _: &mut Vec<Contact<N, P, V>>) {
    }
}
