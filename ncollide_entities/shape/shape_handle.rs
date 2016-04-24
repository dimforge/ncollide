use std::ops::Deref;
use std::sync::Arc;
use inspection::Shape;

/// A shared immutable handle to an abstract shape.
#[derive(Clone)]
pub struct ShapeHandle<P, M> {
    handle: Arc<Box<Shape<P, M>>>
}

impl<P, M> ShapeHandle<P, M> {
    /// Creates a sharable shape handle from a shape.
    #[inline]
    pub fn new<S: Shape<P, M>>(shape: S) -> ShapeHandle<P, M> {
        ShapeHandle {
            handle: Arc::new(Box::new(shape) as Box<Shape<P, M>>)
        }
    }
}

impl<P, M> AsRef<Shape<P, M>> for ShapeHandle<P, M> {
    #[inline]
    fn as_ref(&self) -> &Shape<P, M> {
        self.deref()
    }
}

impl<P, M> Deref for ShapeHandle<P, M> {
    type Target = Shape<P, M>;

    #[inline]
    fn deref(&self) -> &Shape<P, M> {
        &**self.handle
    }
}
