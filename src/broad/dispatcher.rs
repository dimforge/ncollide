/// Trait of dispatcher.
///
/// A (collision) dispatcher typically takes two bodies in parameter and return the corresponding
/// narrow phase algorithm.
pub trait Dispatcher<G1, G2, NF> {
    /// Deduce the narrow phase from two bodies.
    fn dispatch(&self, &G1, &G2) -> NF;
    /// Tells whether a collision between two bodies can occur.
    fn is_valid(&self, &G1, &G2) -> bool;
}

/// Dispatcher which disallows dispatches between identical pointers.
///
/// It has no result.
pub struct NoIdDispatcher<B>;

impl<B> Dispatcher<B, B, ()> for NoIdDispatcher<B> {
    fn dispatch(&self, _: &B, _: &B) -> () {
        ()
    }

    fn is_valid(&self, a: &B, b: &B) -> bool {
        a as *B != b as *B
    }
}
