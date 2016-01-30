use num::Signed;
use na;
use na::{Rotate, Transform};
use support_map::SupportMap;
use shape::Capsule;
use math::{Point, Vect};


impl<P, M> SupportMap<P, M> for Capsule<<P::Vect as Vect>::Scalar>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
    #[inline]
    fn support_point(&self, m: &M, dir: &P::Vect) -> P {
        let local_dir = m.inv_rotate(dir);

        let mut pres = na::orig::<P>();

        if local_dir[1].is_negative() {
            pres[1] = -self.half_height()
        }
        else {
            pres[1] = self.half_height()
        }

        m.transform(&(pres + na::normalize(&local_dir) * self.radius()))
    }
}
