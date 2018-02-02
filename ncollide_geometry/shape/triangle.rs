//! Definition of the triangle shape.

use std::mem;
use na::{self, Point3, Unit};
use shape::{BaseMeshElement, SupportMap};
use math::{Isometry, Point};
use utils;

/// A triangle shape.
#[derive(PartialEq, Debug, Clone, RustcEncodable, RustcDecodable)]
pub struct Triangle<P> {
    a: P,
    b: P,
    c: P,
}

impl<P: Point> Triangle<P> {
    /// Creates a triangle from three points.
    #[inline]
    pub fn new(a: P, b: P, c: P) -> Triangle<P> {
        assert!(na::dimension::<P::Vector>() > 1);
        Triangle { a, b, c }
    }

    /// Creates the reference to a triangle from the reference to an array of three points.
    pub fn from_array(arr: &[P; 3]) -> &Triangle<P> {
        unsafe { mem::transmute(arr) }
    }

    pub(crate) fn from_array4(arr: &[P; 4]) -> &Triangle<P> {
        unsafe { mem::transmute(arr) }
    }

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

    /// The normal of this triangle assuming it is oriented ccw.
    ///
    /// The normal points such that it is collinear to `AB × AC` (where `×` denotes the cross
    /// product).
    #[inline]
    pub fn normal(&self) -> Unit<P::Vector> {
        Unit::new_normalize(self.scaled_normal())
    }

    /// A vector normal of this triangle.
    ///
    /// The vector points such that it is collinear to `AB × AC` (where `×` denotes the cross
    /// product).
    #[inline]
    pub fn scaled_normal(&self) -> P::Vector {
        let ab = self.b - self.a;
        let ac = self.c - self.a;
        utils::cross3(&ab, &ac)
    }
}

impl<P: Point> BaseMeshElement<Point3<usize>, P> for Triangle<P> {
    #[inline]
    fn new_with_vertices_and_indices(vs: &[P], is: &Point3<usize>) -> Triangle<P> {
        Triangle::new(vs[is.x], vs[is.y], vs[is.z])
    }
}

impl<P: Point, M: Isometry<P>> SupportMap<P, M> for Triangle<P> {
    #[inline]
    fn support_point(&self, m: &M, dir: &P::Vector) -> P {
        let local_dir = m.inverse_rotate_vector(dir);

        let d1 = na::dot(&self.a().coordinates(), &local_dir);
        let d2 = na::dot(&self.b().coordinates(), &local_dir);
        let d3 = na::dot(&self.c().coordinates(), &local_dir);

        let res = if d1 > d2 {
            if d1 > d3 {
                self.a()
            } else {
                self.c()
            }
        } else {
            if d2 > d3 {
                self.b()
            } else {
                self.c()
            }
        };

        m.transform_point(res)
    }
}
