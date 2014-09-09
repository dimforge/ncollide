use nalgebra::na::{Transform, Rotate};
use nalgebra::na;
use implicit::{Implicit, PreferedSamplingDirections};
use geom::Triangle;
use math::{Scalar, Vect};

impl<_M: Transform<Vect> + Rotate<Vect>>
Implicit<Vect, _M> for Triangle {
    #[inline]
    fn support_point(&self, m: &_M, dir: &Vect) -> Vect {
        let local_dir = m.inv_rotate(dir);

        let d1 = na::dot(self.a(), &local_dir);
        let d2 = na::dot(self.b(), &local_dir);
        let d3 = na::dot(self.c(), &local_dir);

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

impl<_V, _M> PreferedSamplingDirections<_V, _M> for Triangle {
    #[inline(always)]
    fn sample(&self, _: &_M, _: |_V| -> ()) {
    }
}
