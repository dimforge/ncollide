/// SupportMap representation of the reflection of a shape.
///
/// A reflection is obtained with the central symmetry with regard to the origin.
#[derive(Show)]
pub struct Reflection<'a, Sized? G: 'a> {
    shape: &'a G
}

impl<'a, Sized? G> Reflection<'a, G> {
    /// Build the reflection of a shape. Since the representation is implicit,
    /// the reflection computation is done in constant time.
    #[inline]
    pub fn new(shape: &'a G) -> Reflection<'a, G> {
        Reflection { shape: shape }
    }

    /// The reflected shape.
    #[inline]
    pub fn shape(&self) -> &'a G {
        self.shape
    }
}
