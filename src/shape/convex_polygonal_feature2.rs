use na::{self, RealField, Unit};

use crate::math::{Isometry, Point, Vector};
use crate::query::ContactPreprocessor;
use crate::query::{Contact, ContactPrediction};
use crate::query::{ContactKinematic, ContactManifold, NeighborhoodGeometry};
use crate::shape::{FeatureId, Segment, SegmentPointLocation};

/// A feature (face or vertex) of a 2D convex polygon.
#[derive(Clone, Debug)]
pub struct ConvexPolygonalFeature<N: RealField + Copy> {
    // FIXME: don't keep all those public.
    /// The vertices of this face.
    pub vertices: [Point<N>; 2],
    /// Number of vertices in `vertices` to be considered.
    pub nvertices: usize,
    /// The normal of this feature if it is an edge.
    pub normal: Option<Unit<Vector<N>>>,
    /// The shape-dependent identifier of this feature.
    pub feature_id: FeatureId,
    /// The shape-dependent indentifier of each vertex of this feature.
    pub vertices_id: [FeatureId; 2],
}

impl<N: RealField + Copy> ConvexPolygonalFeature<N> {
    /// Creates a new empty convex polygonal faces.
    pub fn new() -> Self {
        ConvexPolygonalFeature {
            vertices: [Point::origin(); 2],
            nvertices: 0,
            normal: None,
            feature_id: FeatureId::Unknown,
            vertices_id: [FeatureId::Unknown, FeatureId::Unknown],
        }
    }

    /// Removes all the vertices, normals, and feature IDs of this feature.
    pub fn clear(&mut self) {
        self.nvertices = 0;
        self.normal = None;
        self.feature_id = FeatureId::Unknown;
    }

    /// Transforms all the vertices and normal of this feature by the given isometry.
    pub fn transform_by(&mut self, m: &Isometry<N>) {
        for p in &mut self.vertices {
            *p = m * *p;
        }

        if let Some(ref mut n) = self.normal {
            *n = m * *n;
        }
    }

    /// Adds a vertex to this face.
    ///
    /// It is not checked whether `pt` breaks the convexity of the polyhedral face.
    pub fn push(&mut self, pt: Point<N>, id: FeatureId) {
        self.vertices[self.nvertices] = pt;
        self.vertices_id[self.nvertices] = id;
        self.nvertices += 1;
    }

    /// The number of vertices of this face.
    pub fn nvertices(&self) -> usize {
        self.nvertices
    }

    /// The vertices of this convex polygonal face.
    pub fn vertices(&self) -> &[Point<N>] {
        &self.vertices[..self.nvertices]
    }

    /// Sets the outward normal of this convex polygonal face.
    pub fn set_normal(&mut self, normal: Unit<Vector<N>>) {
        self.normal = Some(normal)
    }

    /// Sets the identifier of the feature represented by this convex polygon.
    pub fn set_feature_id(&mut self, id: FeatureId) {
        self.feature_id = id
    }

    /// Projects a point on this feature.
    pub fn project_point(&self, pt: &Point<N>) -> Option<Contact<N>> {
        if let Some(n) = self.normal {
            let dir = self.vertices[1] - self.vertices[0];
            let dpt = *pt - self.vertices[0];
            let dot = dir.dot(&dpt);

            if dot < N::zero() || dot * dot > dir.norm_squared() {
                None
            } else {
                let dist = n.dot(&dpt);
                let proj = *pt + (-n.into_inner() * dist);

                Some(Contact::new(proj, *pt, n, -dist))
            }
        } else {
            None
        }
    }

    /// Generate contacts between `self` and `other` using polygonal clipping, iif. they both have at least
    /// two vertices.
    ///
    /// If either `self` or `other` has less than two vertices, this does nothing.
    pub fn clip(
        &self,
        other: &Self,
        normal: &Unit<Vector<N>>,
        prediction: &ContactPrediction<N>,
        out: &mut Vec<(Contact<N>, FeatureId, FeatureId)>,
    ) {
        // XXX: lift this restriction.
        if self.nvertices <= 1 || other.nvertices <= 1 {
            return;
        }
        // In 2D we always end up with two points.
        let mut ortho: Vector<N> = na::zero();
        ortho[0] = -normal.as_ref()[1];
        ortho[1] = normal.as_ref()[0];

        let mut seg1 = Segment::new(self.vertices[0], self.vertices[1]);
        let mut seg2 = Segment::new(other.vertices[0], other.vertices[1]);

        let ref_pt = seg1.a;
        let mut range1 = [(seg1.a - ref_pt).dot(&ortho), (seg1.b - ref_pt).dot(&ortho)];
        let mut range2 = [(seg2.a - ref_pt).dot(&ortho), (seg2.b - ref_pt).dot(&ortho)];
        let mut features1 = [self.vertices_id[0], self.vertices_id[1]];
        let mut features2 = [other.vertices_id[0], other.vertices_id[1]];

        if range1[1] < range1[0] {
            range1.swap(0, 1);
            features1.swap(0, 1);
            seg1.swap();
        }

        if range2[1] < range2[0] {
            range2.swap(0, 1);
            features2.swap(0, 1);
            seg2.swap();
        }

        if range2[0] > range1[1] || range1[0] > range2[1] {
            return;
        }

        let _1: N = na::one();
        let length1 = range1[1] - range1[0];
        let length2 = range2[1] - range2[0];

        if range2[0] > range1[0] {
            let bcoord = (range2[0] - range1[0]) / length1;
            let p1 = seg1.point_at(&SegmentPointLocation::OnEdge([_1 - bcoord, bcoord]));
            let p2 = seg2.a;
            let contact = Contact::new_wo_depth(p1, p2, *normal);

            if -contact.depth <= prediction.linear() {
                out.push((contact, self.feature_id, features2[0]));
            }
        } else {
            let bcoord = (range1[0] - range2[0]) / length2;
            let p1 = seg1.a;
            let p2 = seg2.point_at(&SegmentPointLocation::OnEdge([_1 - bcoord, bcoord]));
            let contact = Contact::new_wo_depth(p1, p2, *normal);

            if -contact.depth <= prediction.linear() {
                out.push((contact, features1[0], other.feature_id));
            }
        }

        if range2[1] < range1[1] {
            let bcoord = (range2[1] - range1[0]) / length1;
            let p1 = seg1.point_at(&SegmentPointLocation::OnEdge([_1 - bcoord, bcoord]));
            let p2 = seg2.b;
            let contact = Contact::new_wo_depth(p1, p2, *normal);

            if -contact.depth <= prediction.linear() {
                out.push((contact, self.feature_id, features2[1]));
            }
        } else {
            let bcoord = (range1[1] - range2[0]) / length2;
            let p1 = seg1.b;
            let p2 = seg2.point_at(&SegmentPointLocation::OnEdge([_1 - bcoord, bcoord]));
            let contact = Contact::new_wo_depth(p1, p2, *normal);

            if -contact.depth <= prediction.linear() {
                out.push((contact, features1[1], other.feature_id));
            }
        }
    }

    /// Given a contact between two polygonal features, adds it to a contact manifold.
    pub fn add_contact_to_manifold(
        &self,
        other: &Self,
        c: Contact<N>,
        m1: &Isometry<N>,
        f1: FeatureId,
        proc1: Option<&dyn ContactPreprocessor<N>>,
        m2: &Isometry<N>,
        f2: FeatureId,
        proc2: Option<&dyn ContactPreprocessor<N>>,
        manifold: &mut ContactManifold<N>,
    ) {
        let mut kinematic = ContactKinematic::new();
        let local1 = m1.inverse_transform_point(&c.world1);
        let local2 = m2.inverse_transform_point(&c.world2);

        match f1 {
            FeatureId::Face(..) => kinematic.set_approx1(
                f1,
                local1,
                NeighborhoodGeometry::Plane(
                    m1.inverse_transform_unit_vector(&self.normal.as_ref().unwrap()),
                ),
            ),
            FeatureId::Vertex(..) => kinematic.set_approx1(f1, local1, NeighborhoodGeometry::Point),
            FeatureId::Unknown => return,
        }

        match f2 {
            FeatureId::Face(..) => {
                let approx2 = NeighborhoodGeometry::Plane(
                    m2.inverse_transform_unit_vector(other.normal.as_ref().unwrap()),
                );
                kinematic.set_approx2(f2, local2, approx2)
            }
            FeatureId::Vertex(..) => kinematic.set_approx2(f2, local2, NeighborhoodGeometry::Point),
            FeatureId::Unknown => return,
        }

        let _ = manifold.push(c, kinematic, local1, proc1, proc2);
    }
}
