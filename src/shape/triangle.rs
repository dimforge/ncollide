//! Definition of the triangle shape.

use std::mem;
use approx::ApproxEq;
use na::{self, Point3, Unit};
use na::Real;
use shape::{BaseMeshElement, SupportMap};
use math::{Isometry, Point};
use utils;

/// A triangle shape.
#[derive(PartialEq, Debug, Clone)]
pub struct Triangle<P> {
    a: P,
    b: P,
    c: P,
}

/// Description of the location of a point on a triangle.
#[derive(Copy, Clone, Debug)]
pub enum TrianglePointLocation<N: Real> {
    /// The point lies on a vertex.
    OnVertex(usize),
    /// The point lies on an edge.
    OnEdge(usize, [N; 2]),
    /// The point lies on the triangle interior.
    OnFace([N; 3]),
    /// The point lies on the triangle interior (for "solid" point queries).
    OnSolid,
}

impl<N: Real> TrianglePointLocation<N> {
    /// Returns `true` if the point is located on the relative interior of the triangle.
    pub fn is_on_face(&self) -> bool {
        if let TrianglePointLocation::OnFace(_) = *self {
            true
        } else {
            false
        }
    }
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
    pub fn normal(&self) -> Option<Unit<P::Vector>> {
        Unit::try_new(self.scaled_normal(), P::Real::default_epsilon())
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

// impl<P: Point, M: Isometry<P>> ConvexPolyhedron<P, M> for Triangle<P> {
//     fn vertex(&self, id: FeatureId) -> P {
//         match id.unwrap_vertex() {
//             0 => self.a,
//             1 => self.b,
//             2 => self.c,
//             _ => panic!("Triangle vertex index out of bounds."),
//         }
//     }
//     fn edge(&self, id: FeatureId) -> (P, P, FeatureId, FeatureId) {
//         match id.unwrap_edge() {
//             0 => (self.a, self.b, FeatureId::Vertex(0), FeatureId::Vertex(1)),
//             2 => (self.b, self.c, FeatureId::Vertex(1), FeatureId::Vertex(2)),
//             3 => (self.c, self.a, FeatureId::Vertex(2), FeatureId::Vertex(0)),
//             _ => panic!("Triangle edge index out of bounds."),
//         }
//     }

//     fn face(&self, id: FeatureId, face: &mut ConvexPolyface<P>) {
//         if na::dimension::<P::Vector>() != 2 {
//             panic!("A segment does not have any face indimensions higher than 2.")
//         }

//         face.clear();

//         if let Some(normal) = self.normal() {
//             face.set_feature_id(id);

//             match id.unwrap_face() {
//                 0 => {
//                     face.push(self.a, FeatureId::Vertex(0));
//                     face.push(self.b, FeatureId::Vertex(1));
//                     face.push(self.c, FeatureId::Vertex(2));
//                     face.set_normal(normal);
//                 }
//                 1 => {
//                     face.push(self.a, FeatureId::Vertex(0));
//                     face.push(self.c, FeatureId::Vertex(2));
//                     face.push(self.b, FeatureId::Vertex(1));
//                     face.set_normal(-normal);
//                 }
//                 _ => unreachable!(),
//             }
//         } else {
//             face.push(self.a, FeatureId::Vertex(0));
//             face.set_feature_id(FeatureId::Vertex(0));
//         }
//     }

//     fn normal_cone(&self, feature: FeatureId) -> PolyhedralCone<P::Vector> {
//         if let Some(direction) = self.direction() {
//             match feature {
//                 FeatureId::Vertex(id) => {
//                     if id == 0 {
//                         PolyhedralCone::HalfSpace(direction)
//                     } else {
//                         PolyhedralCone::HalfSpace(-direction)
//                     }
//                 }
//                 FeatureId::Edge(_) => PolyhedralCone::OrthogonalSubspace(direction),
//                 FeatureId::Face(id) => {
//                     assert!(na::dimension::<P::Vector>() == 2);

//                     let mut dir = P::Vector::zero();
//                     if id == 0 {
//                         dir[0] = direction[1];
//                         dir[1] = -direction[0];
//                     } else {
//                         dir[0] = -direction[1];
//                         dir[1] = direction[0];
//                     }
//                     PolyhedralCone::HalfLine(Unit::new_unchecked(dir))
//                 }
//                 _ => PolyhedralCone::Empty,
//             }
//         } else {
//             PolyhedralCone::Full
//         }
//     }

//     fn support_face_toward(&self, m: &M, dir: &Unit<P::Vector>, face: &mut ConvexPolyface<P>) {
//         assert!(na::dimension::<P::Vector>() == 3);
//         let normal = self.scaled_normal();

//         if na::dot(&normal, &*dir) >= na::zero() {
//             ConvexPolyhedron::<P, M>::face(self, FeatureId::Face(0), face);
//         } else {
//             ConvexPolyhedron::<P, M>::face(self, FeatureId::Face(1), face);
//         }
//         face.transform_by(m)
//     }

//     fn support_feature_toward(
//         &self,
//         transform: &M,
//         dir: &Unit<P::Vector>,
//         _angle: P::Real,
//         out: &mut ConvexPolyface<P>,
//     ) {
//         out.clear();
//         // FIXME: actualy find the support feature.
//         self.support_face_toward(transform, dir, out)
//     }

//     fn support_feature_id_toward(&self, local_dir: &Unit<P::Vector>) -> FeatureId {
//         if let Some(normal) = self.normal() {
//             let eps: P::Real = na::convert(f64::consts::PI / 180.0);
//             let (seps, ceps) = eps.sin_cos();

//             let normal_dot = na::dot(&*local_dir, &*normal);
//             if normal_dot >= ceps {
//                 FeatureId::Face(0)
//             } else if normal_dot <= -ceps {
//                 FeatureId::Face(1)
//             }
//         } else {
//             unimplemented!()
//         }
//     }
// }
