use num::Signed;
use na::{Rotate, Transform, Norm};
use na;
use support_map::SupportMap;
use shape::Cylinder;
use math::{Scalar, Point, Vect};



impl<P, M> SupportMap<P, M> for Cylinder<<P::Vect as Vect>::Scalar>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
    fn support_point(&self, m: &M, dir: &P::Vect) -> P {
        let local_dir = m.inv_rotate(dir);

        let mut vres = local_dir.clone();

        let negative = local_dir[1].is_negative();

        vres[1]  = na::zero();

        if na::is_zero(&vres.normalize_mut()) {
            vres = na::zero()
        }
        else {
            vres = vres * self.radius();
        }

        if negative {
            vres[1] = -self.half_height()
        }
        else {
            vres[1] = self.half_height()
        }

        m.transform(&(na::orig::<P>() + vres))
    }
}
