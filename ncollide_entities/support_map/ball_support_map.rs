use na;
use na::Translate;
use support_map::SupportMap;
use shape::Ball;
use math::{Point, Vector};


impl< P, M> SupportMap<P, M> for Ball<<P::Vect as Vector>::Scalar>
    where P: Point,
          M: Translate<P> {
    #[inline]
    fn support_point(&self, m: &M, dir: &P::Vect) -> P {
        m.translate(&na::origin()) + na::normalize(dir) * self.radius()
    }
}
