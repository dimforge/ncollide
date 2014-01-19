/// Implicit representation of the reflection of a geometry.
///
/// A reflection is obtained with the central symmetry with regard to the origin.
#[deriving(Eq, ToStr, Clone)]
pub struct Reflection<'a, G> {
    priv g: &'a G
}

impl<'a, G> Reflection<'a, G> {
    /// Build the reflection of a geometry. Since the representation is implicit,
    /// the reflection computation is done in constant time.
    #[inline]
    pub fn new(g: &'a G) -> Reflection<'a, G> {
        Reflection { g: g }
    }

    /// The reflected geometry.
    #[inline]
    pub fn g(&self) -> &'a G {
        self.g
    }
}
