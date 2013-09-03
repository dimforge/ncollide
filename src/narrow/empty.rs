use narrow::CollisionDetector;
use contact::Contact;

/// An empty collision detector. It does nothing.
pub struct Empty<N, V, M, G1, G2> {
    priv dummy: uint // FIXME: useless, but zero-sized structure ICE when used cross-crate.
}

impl<N, V, M, G1, G2> Empty<N, V, M, G1, G2> {
    /// Creates a new empty collision detector.
    pub fn new() -> Empty<N, V, M, G1, G2> {
        Empty {
            dummy: 0
        }
    }
}

impl<N, V, M, G1, G2> CollisionDetector<N, V, M, G1, G2> for Empty<N, V, M, G1, G2> {
    fn update(&mut self, _: &M, _: &G1, _: &M, _: &G2) {
    }

    fn num_colls(&self) -> uint {
        0
    }

    fn colls(&self, _: &mut ~[Contact<N, V>]) {
    }

    fn toi(_: Option<Empty<N, V, M, G1, G2>>, _: &M, _: &V, _: &N, _: &G1, _: &M, _: &G2) -> Option<N> {
        None
    }
}
