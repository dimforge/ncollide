use na::{Transform, Rotate};
use na;
use shape::Cuboid;
use support_map::SupportMap;
use math::Point;


impl<P, M> SupportMap<P, M> for Cuboid<P::Vect>
    where P: Point,
          M: Rotate<P::Vect> + Transform<P> {
    #[inline]
    fn support_point(&self, m: &M, dir: &P::Vect) -> P {
        let local_dir = m.inv_rotate(dir);

        let mut pres: P = na::orig();

        let he = self.half_extents();
        for i in 0usize .. na::dim::<P>() {
            if local_dir[i] < na::zero() {
                pres[i] = -he[i];
            }
            else {
                pres[i] = he[i];
            }
        }

        m.transform(&pres)
    }
}
