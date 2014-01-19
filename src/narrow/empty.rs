use narrow::CollisionDetector;
use contact::Contact;
use math::{N, V, M};

/// A collision detector that does nothing.
#[deriving(Encodable, Decodable)]
pub struct Empty<G1, G2> {
    priv dummy: uint // FIXME: useless, but zero-sized structure ICE when used cross-crate.
}

impl<G1, G2> Empty<G1, G2> {
    /// Creates a new empty collision detector.
    pub fn new() -> Empty<G1, G2> {
        Empty {
            dummy: 0
        }
    }
}

impl<G1, G2> CollisionDetector<G1, G2> for Empty<G1, G2> {
    fn update(&mut self, _: &M, _: &G1, _: &M, _: &G2) {
    }

    fn num_colls(&self) -> uint {
        0
    }

    fn colls(&self, _: &mut ~[Contact]) {
    }

    fn toi(_: Option<Empty<G1, G2>>, _: &M, _: &V, _: &N, _: &G1, _: &M, _: &G2) -> Option<N> {
        None
    }
}
