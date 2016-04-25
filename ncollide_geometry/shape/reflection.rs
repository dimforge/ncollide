use shape::SupportMap;
use math::Point;

/// SupportMap representation of the reflection of a shape.
///
/// A reflection is obtained with the central symmetry with regard to the origin.
#[derive(Debug)]
pub struct Reflection<'a, G: ?Sized + 'a> {
    shape: &'a G
}

impl<'a, G: ?Sized> Reflection<'a, G> {
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

impl<'a, P, M, G: ?Sized> SupportMap<P, M> for Reflection<'a, G>
    where P: Point,
          G: SupportMap<P, M> {
    #[inline]
    fn support_point(&self, m: &M, dir: &P::Vect) -> P {
        -self.shape().support_point(m, &-*dir)
    }
}
