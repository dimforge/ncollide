use std::managed;

/// Trait of dispatcher. A (collision) dispatcher typically takes two bodies in parameter and
/// return the corresponding narrow phase algorithm.
pub trait Dispatcher<B, NF>
{
    /// Deduce the narrow phase from two bodies.
    fn dispatch(&self, @mut B, @mut B) -> NF;
    /// Tells whether a collision between two bodies can occur.
    fn is_valid(&self, @mut B, @mut B) -> bool;
}

/// Dispatcher which disallows dispatches between identical pointers. It has no result.
pub struct NoIdDispatcher<B>;

impl<B> Dispatcher<B, ()> for NoIdDispatcher<B>
{
    fn dispatch(&self, _: @mut B, _: @mut B) -> ()
    { () }

    fn is_valid(&self, a: @mut B, b: @mut B) -> bool
    { !managed::mut_ptr_eq(a, b) }
}
