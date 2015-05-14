use support_map::SupportMap;
use shape::Reflection;
use math::{Point, Vect};


impl<'a, P, M, G: ?Sized> SupportMap<P, M> for Reflection<'a, G>
    where P: Point,
          G: SupportMap<P, M> {
    #[inline]
    fn support_point(&self, m: &M, dir: &P::Vect) -> P {
        -self.shape().support_point(m, &-*dir)
    }
}
