/// Implicit representation of the reflection of a shape.
///
/// A reflection is obtained with the central symmetry with regard to the origin.
#[deriving(Show)]
pub struct Reflection<'a, G: 'a> {
    geom: &'a G
}

impl<'a, G> Reflection<'a, G> {
    /// Build the reflection of a shape. Since the representation is implicit,
    /// the reflection computation is done in constant time.
    #[inline]
    pub fn new(geom: &'a G) -> Reflection<'a, G> {
        Reflection { geom: geom }
    }

    /// The reflected shape.
    #[inline]
    pub fn geom(&self) -> &'a G {
        self.geom
    }
}
