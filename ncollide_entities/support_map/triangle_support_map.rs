use na::{Transform, Rotate};
use na;
use support_map::SupportMap;
use shape::Triangle;
use math::Point;


impl<P, M> SupportMap<P, M> for Triangle<P>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
    #[inline]
    fn support_point(&self, m: &M, dir: &P::Vect) -> P {
        let local_dir = m.inv_rotate(dir);

        let d1 = na::dot(self.a().as_vec(), &local_dir);
        let d2 = na::dot(self.b().as_vec(), &local_dir);
        let d3 = na::dot(self.c().as_vec(), &local_dir);

        let res =
            if d1 > d2 {
                if d1 > d3 {
                    self.a()
                }
                else {
                    self.c()
                }
            }
            else {
                if d2 > d3 {
                    self.b()
                }
                else {
                    self.c()
                }
            };

        m.transform(res)
    }
}
