use na::{Transform, Rotate};
use na;
use support_map::SupportMap;
use shape::Segment;
use math::Point;


impl<P, M> SupportMap<P, M> for Segment<P>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
    #[inline]
    fn support_point(&self, m: &M, dir: &P::Vect) -> P {
        let local_dir = m.inverse_rotate(dir);

        if na::dot(self.a().as_vector(), &local_dir) > na::dot(self.b().as_vector(), &local_dir) {
            m.transform(self.a())
        }
        else {
            m.transform(self.b())
        }
    }
}
