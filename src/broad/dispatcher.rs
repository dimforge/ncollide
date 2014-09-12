/// Trait of dispatcher.
///
/// A (collision) dispatcher typically takes two bodies in parameter and return the corresponding
/// narrow phase algorithm.
pub trait Dispatcher<G1, G2, NF> {
    /// Deduce the narrow phase from two bodies.
    fn dispatch(&self, &G1, &G2) -> Option<NF>;
    /// Tells whether a collision between two bodies can occur.
    fn is_valid(&self, &G1, &G2) -> bool;
}

/// Dispatcher which disallows dispatches between identical pointers.
///
/// It has no result.
pub struct NoIdDispatcher<B>;

impl<B> NoIdDispatcher<B> {
    /// Creates a new `NoIdDispatcher`.
    pub fn new() -> NoIdDispatcher<B> {
        NoIdDispatcher
    }
}

impl<B> Dispatcher<B, B, ()> for NoIdDispatcher<B> {
    #[inline]
    fn dispatch(&self, _: &B, _: &B) -> Option<()> {
        Some(())
    }

    #[inline]
    fn is_valid(&self, a: &B, b: &B) -> bool {
        a as *const B != b as *const B
    }
}
