//! Definition of the segment shape.

use std::f64;
use std::mem;
use num::Zero;
use approx::ApproxEq;
use na::{self, Point2, Real, Unit};
use utils;
use bounding_volume::PolyhedralCone;
use shape::{BaseMeshElement, ConvexPolyface, ConvexPolyhedron, FeatureId, SupportMap};
use math::{Isometry, Point};

/// A segment shape.
#[derive(PartialEq, Debug, Clone)]
pub struct Segment<P> {
    a: P,
    b: P,
}

/// Logical description of the location of a point on a triangle.
pub enum SegmentPointLocation<N: Real> {
    /// The point lies on a vertex.
    OnVertex(usize),
    /// The point lies on the segment interior.
    OnEdge([N; 2]),
}

impl<P: Point> Segment<P> {
    /// Creates a new segment from two points.
    #[inline]
    pub fn new(a: P, b: P) -> Segment<P> {
        assert!(na::dimension::<P::Vector>() > 1);
        Segment { a, b }
    }

    /// Creates the reference to a segment from the reference to an array of two points.
    pub fn from_array(arr: &[P; 2]) -> &Segment<P> {
        unsafe { mem::transmute(arr) }
    }

    pub(crate) fn from_array3(arr: &[P; 3]) -> &Segment<P> {
        unsafe { mem::transmute(arr) }
    }

    pub(crate) fn from_array4(arr: &[P; 4]) -> &Segment<P> {
        unsafe { mem::transmute(arr) }
    }
}

impl<P> Segment<P> {
    /// The first point of this segment.
    #[inline]
    pub fn a(&self) -> &P {
        &self.a
    }

    /// The second point of this segment.
    #[inline]
    pub fn b(&self) -> &P {
        &self.b
    }
}

impl<P: Point> Segment<P> {
    /// The direction of this segment scaled by its length.
    ///
    /// Points from `self.a()` toward `self.b()`.
    pub fn scaled_direction(&self) -> P::Vector {
        self.b - self.a
    }

    /// The length of this segment.
    pub fn length(&self) -> P::Real {
        na::norm(&self.scaled_direction())
    }

    /// Swaps the two vertices of this segment.
    pub fn swap(&mut self) {
        mem::swap(&mut self.a, &mut self.b)
    }

    /// The unit direction of this segment.
    ///
    /// Points from `self.a()` toward `self.b()`.
    /// Returns `None` is both points are equal.
    pub fn direction(&self) -> Option<Unit<P::Vector>> {
        Unit::try_new(self.scaled_direction(), P::Real::default_epsilon())
    }

    /// Applies the isometry `m` to the vertices of this segment and returns the resulting segment.
    pub fn transformed<M: Isometry<P>>(&self, m: &M) -> Self {
        Segment::new(m.transform_point(&self.a), m.transform_point(&self.b))
    }

    /// Computes the point at the given location.
    pub fn point_at(&self, location: &SegmentPointLocation<P::Real>) -> P {
        match *location {
            SegmentPointLocation::OnVertex(0) => self.a,
            SegmentPointLocation::OnVertex(1) => self.b,
            SegmentPointLocation::OnEdge(bcoords) => {
                let mut res = self.a;
                res.axpy(bcoords[1], &self.b, bcoords[0]);
                res
            }
            _ => unreachable!(),
        }
    }
}

impl<P: Point> BaseMeshElement<Point2<usize>, P> for Segment<P> {
    #[inline]
    fn new_with_vertices_and_indices(vs: &[P], is: &Point2<usize>) -> Segment<P> {
        Segment::new(vs[is.x], vs[is.y])
    }
}

impl<P: Point, M: Isometry<P>> SupportMap<P, M> for Segment<P> {
    #[inline]
    fn support_point(&self, m: &M, dir: &P::Vector) -> P {
        let local_dir = m.inverse_transform_vector(dir);

        if na::dot(&self.a().coordinates(), &local_dir)
            > na::dot(&self.b().coordinates(), &local_dir)
        {
            m.transform_point(self.a())
        } else {
            m.transform_point(self.b())
        }
    }
}

impl<P: Point, M: Isometry<P>> ConvexPolyhedron<P, M> for Segment<P> {
    fn vertex(&self, id: FeatureId) -> P {
        if id.unwrap_vertex() == 0 {
            self.a
        } else {
            self.b
        }
    }
    fn edge(&self, _: FeatureId) -> (P, P, FeatureId, FeatureId) {
        (self.a, self.b, FeatureId::Vertex(0), FeatureId::Vertex(1))
    }
    fn face(&self, id: FeatureId, face: &mut ConvexPolyface<P>) {
        if na::dimension::<P::Vector>() != 2 {
            panic!("A segment does not have any face indimensions higher than 2.")
        }

        face.clear();

        if let Some(normal) = P::ccw_face_normal(&[&self.a, &self.b]) {
            face.set_feature_id(id);

            match id.unwrap_face() {
                0 => {
                    face.push(self.a, FeatureId::Vertex(0));
                    face.push(self.b, FeatureId::Vertex(1));
                    face.set_normal(normal);
                }
                1 => {
                    face.push(self.b, FeatureId::Vertex(1));
                    face.push(self.a, FeatureId::Vertex(0));
                    face.set_normal(-normal);
                }
                _ => unreachable!(),
            }
        } else {
            face.push(self.a, FeatureId::Vertex(0));
            face.set_feature_id(FeatureId::Vertex(0));
        }
    }

    fn normal_cone(&self, feature: FeatureId) -> PolyhedralCone<P::Vector> {
        if let Some(direction) = self.direction() {
            match feature {
                FeatureId::Vertex(id) => {
                    if id == 0 {
                        PolyhedralCone::HalfSpace(direction)
                    } else {
                        PolyhedralCone::HalfSpace(-direction)
                    }
                }
                FeatureId::Edge(_) => PolyhedralCone::OrthogonalSubspace(direction),
                FeatureId::Face(id) => {
                    assert!(na::dimension::<P::Vector>() == 2);

                    let mut dir = P::Vector::zero();
                    if id == 0 {
                        dir[0] = direction[1];
                        dir[1] = -direction[0];
                    } else {
                        dir[0] = -direction[1];
                        dir[1] = direction[0];
                    }
                    PolyhedralCone::HalfLine(Unit::new_unchecked(dir))
                }
                _ => PolyhedralCone::Empty,
            }
        } else {
            PolyhedralCone::Full
        }
    }

    fn support_face_toward(&self, m: &M, dir: &Unit<P::Vector>, face: &mut ConvexPolyface<P>) {
        if na::dimension::<P::Vector>() == 2 {
            let seg_dir = self.scaled_direction();

            if utils::perp2(dir.as_ref(), &seg_dir) >= na::zero() {
                ConvexPolyhedron::<P, M>::face(self, FeatureId::Face(0), face);
            } else {
                ConvexPolyhedron::<P, M>::face(self, FeatureId::Face(1), face);
            }
        } else {
            face.push(self.a, FeatureId::Vertex(0));
            face.push(self.b, FeatureId::Vertex(1));
            face.push_edge_feature_id(FeatureId::Edge(0));
            face.set_feature_id(FeatureId::Edge(0));
        }
        face.transform_by(m)
    }

    fn support_feature_toward(
        &self,
        transform: &M,
        dir: &Unit<P::Vector>,
        _angle: P::Real,
        out: &mut ConvexPolyface<P>,
    ) {
        out.clear();
        // FIXME: actualy find the support feature.
        self.support_face_toward(transform, dir, out)
    }

    fn support_feature_id_toward(&self, local_dir: &Unit<P::Vector>) -> FeatureId {
        if let Some(seg_dir) = self.direction() {
            let eps: P::Real = na::convert(f64::consts::PI / 180.0);
            let seps = eps.sin();
            let dot = na::dot(seg_dir.as_ref(), local_dir.as_ref());

            if dot <= seps {
                if na::dimension::<P::Vector>() == 2 {
                    if utils::perp2(local_dir.as_ref(), seg_dir.as_ref()) >= na::zero() {
                        FeatureId::Face(0)
                    } else {
                        FeatureId::Face(1)
                    }
                } else {
                    FeatureId::Edge(0)
                }
            } else if dot >= na::zero() {
                FeatureId::Vertex(1)
            } else {
                FeatureId::Vertex(0)
            }
        } else {
            FeatureId::Vertex(0)
        }
    }
}
