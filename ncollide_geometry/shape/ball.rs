use na;
use na::Translate;
use shape::SupportMap;
use math::{Scalar, Point, Vector};

/// A Ball shape.
#[derive(PartialEq, Debug, Clone, RustcEncodable, RustcDecodable)]
pub struct Ball<N> {
    radius: N
}

impl<N: Scalar> Ball<N> {
    /// Creates a new ball from its radius and center.
    #[inline]
    pub fn new(radius: N) -> Ball<N> {
        assert!(radius > na::zero(), "A ball radius must be strictly positive.");

        Ball {
            radius: radius
        }
    }

    /// The ball radius.
    #[inline]
    pub fn radius(&self) -> N {
        self.radius.clone()
    }
}

impl<P, M> SupportMap<P, M> for Ball<<P::Vect as Vector>::Scalar>
    where P: Point,
          M: Translate<P> {
    #[inline]
    fn support_point(&self, m: &M, dir: &P::Vect) -> P {
        m.translate(&na::origin()) + na::normalize(dir) * self.radius()
    }

    #[inline]
    fn support_point_set(&self,
                         transform:  &M,
                         dir:        &P::Vect,
                         _:          <P::Vect as Vector>::Scalar,
                         _:          usize,
                         out_points: &mut Vec<P>)
                         -> usize {
         let pt = self.support_point(transform, dir);
         out_points.push(pt);

         1
     }
}
