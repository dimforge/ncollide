use num::Signed;
use na::{Rotate, Transform, Norm};
use na;
use support_map::SupportMap;
use shape::Cone;
use math::{Point, Vector};


impl<P, M> SupportMap<P, M> for Cone<<P::Vect as Vector>::Scalar>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
    #[inline]
    fn support_point(&self, m: &M, dir: &P::Vect) -> P {
        let local_dir = m.inverse_rotate(dir);

        let mut vres = local_dir.clone();

        vres[1] = na::zero();

        if na::is_zero(&vres.normalize_mut()) {
            vres = na::zero();

            if local_dir[1].is_negative() {
                vres[1] = -self.half_height()
            }
            else {
                vres[1] = self.half_height()
            }
        }
        else {
            vres = vres * self.radius();
            vres[1] = -self.half_height();

            if na::dot(&local_dir, &vres) < local_dir[1] * self.half_height() {
                vres = na::zero();
                vres[1] = self.half_height()
            }
        }

        m.transform(&(na::origin::<P>() + vres))
    }
}
