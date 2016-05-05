//! Definition of the triangle shape.

use na::{self, Dimension, Transform, Rotate, Point3};
use shape::{BaseMeshElement, SupportMap};
use math::Point;


/// A triangle shape.
#[derive(PartialEq, Debug, Clone, RustcEncodable, RustcDecodable)]
pub struct Triangle<P> {
    a: P,
    b: P,
    c: P
}

impl<P: Dimension> Triangle<P> {
    /// Creates a triangle from three points.
    #[inline]
    pub fn new(a: P, b: P, c: P) -> Triangle<P> {
        assert!(na::dimension::<P>() > 1);

        Triangle {
            a: a,
            b: b,
            c: c
        }
    }
}

impl<P> Triangle<P> {
    /// The fist point of this triangle.
    #[inline]
    pub fn a(&self) -> &P {
        &self.a
    }

    /// The second point of this triangle.
    #[inline]
    pub fn b(&self) -> &P {
        &self.b
    }

    /// The third point of this triangle.
    #[inline]
    pub fn c(&self) -> &P {
        &self.c
    }
}

impl<P: Copy + Dimension> BaseMeshElement<Point3<usize>, P> for Triangle<P> {
    #[inline]
    fn new_with_vertices_and_indices(vs: &[P], is: &Point3<usize>) -> Triangle<P> {
        Triangle::new(vs[is.x], vs[is.y], vs[is.z])
    }
}

impl<P, M> SupportMap<P, M> for Triangle<P>
    where P: Point,
          M: Transform<P> + Rotate<P::Vect> {
    #[inline]
    fn support_point(&self, m: &M, dir: &P::Vect) -> P {
        let local_dir = m.inverse_rotate(dir);

        let d1 = na::dot(self.a().as_vector(), &local_dir);
        let d2 = na::dot(self.b().as_vector(), &local_dir);
        let d3 = na::dot(self.c().as_vector(), &local_dir);

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
