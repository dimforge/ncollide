use std::ops::Deref;
use std::sync::Arc;
use inspection::Repr;

/// A shared immutable handle to an abstract shape.
#[derive(Clone)]
pub struct ShapeHandle<P, M> {
    handle: Arc<Box<Repr<P, M>>>
}

impl<P, M> ShapeHandle<P, M> {
    /// Creates a sharable shape handle from a shape.
    #[inline]
    pub fn new<S: Repr<P, M>>(shape: S) -> ShapeHandle<P, M> {
        ShapeHandle {
            handle: Arc::new(Box::new(shape) as Box<Repr<P, M>>)
        }
    }
}

impl<P, M> AsRef<Repr<P, M>> for ShapeHandle<P, M> {
    #[inline]
    fn as_ref(&self) -> &Repr<P, M> {
        self.deref()
    }
}

impl<P, M> Deref for ShapeHandle<P, M> {
    type Target = Repr<P, M>;

    #[inline]
    fn deref(&self) -> &Repr<P, M> {
        &**self.handle
    }
}
