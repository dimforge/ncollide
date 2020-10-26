use na::{self, Point2, RealField, Unit};
use std::iter;

use crate::math::{Isometry, Point, Vector};
use crate::query::ContactPreprocessor;
use crate::query::{
    self, Contact, ContactKinematic, ContactManifold, ContactPrediction, NeighborhoodGeometry,
};
use crate::shape::{FeatureId, Segment, SegmentPointLocation};
use crate::utils;

/// A cache used for polygonal clipping.
#[derive(Clone)]
pub struct ClippingCache<N: RealField> {
    poly1: Vec<Point2<N>>,
    poly2: Vec<Point2<N>>,
}

impl<N: RealField> ClippingCache<N> {
    /// Initializes an empty clipping cache.
    pub fn new() -> Self {
        ClippingCache {
            poly1: Vec::with_capacity(4),
            poly2: Vec::with_capacity(4),
        }
    }

    /// Clears the clipping cache.
    pub fn clear(&mut self) {
        self.poly1.clear();
        self.poly2.clear();
    }
}

/// Represents a convex polygonal approximation of a face of a solid.
///
/// It is never checked if the vertices actually form a convex polygon.
/// If they do not, results of any geometric query may end up being invalid.
#[derive(Clone, Debug)]
pub struct ConvexPolygonalFeature<N: RealField> {
    // FIXME: don't keep all those public.
    /// The vertices of this face.
    pub vertices: Vec<Point<N>>,
    /// The outward normal of the edges if it is a face.
    pub edge_normals: Vec<Vector<N>>,
    /// The normal of this feature if it is a face.
    pub normal: Option<Unit<Vector<N>>>,
    /// The shape-dependent identifier of this feature.
    pub feature_id: FeatureId,
    /// The shape-dependent indentifier of each vertex of this feature.
    pub vertices_id: Vec<FeatureId>,
    /// The shape-dependent indentifier of each edge of this feature.
    pub edges_id: Vec<FeatureId>,
}

impl<N: RealField> ConvexPolygonalFeature<N> {
    /// Creates a new empty convex polygonal faces.
    pub fn new() -> Self {
        ConvexPolygonalFeature {
            vertices: Vec::new(),
            edge_normals: Vec::new(),
            normal: None,
            feature_id: FeatureId::Unknown,
            vertices_id: Vec::new(),
            edges_id: Vec::new(),
        }
    }

    /// Creates a new convex polygonal feature with all field initialized with `n` zero elements.
    pub fn with_size(n: usize) -> Self {
        ConvexPolygonalFeature {
            vertices: iter::repeat(Point::origin()).take(n).collect(),
            edge_normals: iter::repeat(Vector::zeros()).take(n).collect(),
            normal: None,
            feature_id: FeatureId::Unknown,
            vertices_id: iter::repeat(FeatureId::Unknown).take(n).collect(),
            edges_id: iter::repeat(FeatureId::Unknown).take(n).collect(),
        }
    }

    /// Removes all the vertices, normals, and feature IDs of this feature.
    pub fn clear(&mut self) {
        self.vertices.clear();
        self.edge_normals.clear();
        self.vertices_id.clear();
        self.edges_id.clear();
        self.normal = None;
        self.feature_id = FeatureId::Unknown;
    }

    /// Transforms all the vertices and normals of this feature by the given isometry.
    pub fn transform_by(&mut self, m: &Isometry<N>) {
        for p in &mut self.vertices {
            *p = m * *p;
        }

        for n in &mut self.edge_normals {
            *n = m * *n;
        }

        if let Some(ref mut n) = self.normal {
            *n = m * *n;
        }
    }

    /// Adds a vertex to this face.
    ///
    /// It is not checked whether `pt` breaks the convexity of the polyhedral face.
    pub fn push(&mut self, pt: Point<N>, id: FeatureId) {
        self.vertices.push(pt);
        self.vertices_id.push(id);
    }

    /// The number of vertices of this face.
    pub fn nvertices(&self) -> usize {
        self.vertices.len()
    }

    /// The vertices of this convex polygonal face.
    pub fn vertices(&self) -> &[Point<N>] {
        &self.vertices[..]
    }

    /// The number of edges of this convex polygonal face.
    pub fn nedges(&self) -> usize {
        match self.vertices.len() {
            1 => 0,
            2 => 1,
            l => l,
        }
    }

    /// Retrieves the edge with the given feature id.
    pub fn edge(&self, edge_id: FeatureId) -> Option<Segment<N>> {
        for i1 in 0..self.vertices.len() {
            if self.edges_id[i1] == edge_id {
                let i2 = (i1 + 1) % self.vertices.len();
                return Some(Segment::new(self.vertices[i1], self.vertices[i2]));
            }
        }

        None
    }

    /// Adds a scaled edge normal to this face.
    pub fn push_scaled_edge_normal(&mut self, normal: Vector<N>) {
        if let Some(normal) = normal.try_normalize(N::default_epsilon()) {
            self.edge_normals.push(normal)
        } else {
            self.edge_normals.push(na::zero())
        }
    }

    /// Adds an edge normal to this face.
    pub fn push_edge_normal(&mut self, normal: Unit<Vector<N>>) {
        self.edge_normals.push(normal.into_inner())
    }

    /// Automatically recomputes the scaled edge normals (3D only).
    ///
    /// Panics if the ambient space is not 3D.
    pub fn recompute_edge_normals(&mut self) {
        self.edge_normals.clear();

        for i1 in 0..self.vertices.len() {
            let i2 = (i1 + 1) % self.vertices.len();
            let dpt = self.vertices[i2] - self.vertices[i1];
            let scaled_normal = dpt.cross(
                self.normal
                    .as_ref()
                    .expect("The face normal must be set before computing edge normals."),
            );
            self.push_scaled_edge_normal(scaled_normal)
        }
    }

    /// Transforms all the vertices of this feature by the given isometry.
    pub fn project_point(&self, pt: &Point<N>) -> Option<Contact<N>> {
        if let Some(n) = self.normal {
            let dpt = *pt - self.vertices[0];
            let dist = n.dot(&dpt);
            let proj = *pt + (-n.into_inner() * dist);

            for i in 0..self.edge_normals.len() {
                let dpt = proj - self.vertices[i];

                if dpt.dot(&self.edge_normals[i]) > na::zero() {
                    return None;
                }
            }

            Some(Contact::new(proj, *pt, n, -dist))
        } else {
            None
        }
    }

    /// Sets the outward normal of this convex polygonal face.
    pub fn set_normal(&mut self, normal: Unit<Vector<N>>) {
        self.normal = Some(normal)
    }

    /// Add the shape-dependent identifier of a edge of this feature (if it is a face).
    pub fn push_edge_feature_id(&mut self, id: FeatureId) {
        self.edges_id.push(id)
    }

    /// Add the shape-dependent identifier of this feature.
    pub fn set_feature_id(&mut self, id: FeatureId) {
        self.feature_id = id
    }

    /// Generate contacts between `self` and `other` using polygonal clipping, iif. they both have at least
    /// three vertices.
    ///
    /// If either `self` or `other` has less than three vertices, this does nothing.
    pub fn clip(
        &self,
        other: &Self,
        normal: &Unit<Vector<N>>,
        prediction: &ContactPrediction<N>,
        cache: &mut ClippingCache<N>,
        out: &mut Vec<(Contact<N>, FeatureId, FeatureId)>,
    ) {
        // FIXME: don't compute contacts further than the prediction.

        cache.clear();

        // FIXME: lift this restriction.
        if self.vertices.len() <= 2 && other.vertices.len() <= 2 {
            return;
        }

        // In 3D we may end up with more than two points.
        let mut basis = [na::zero(), na::zero()];
        let mut basis_i = 0;

        Vector::orthonormal_subspace_basis(&[normal.into_inner()], |dir| {
            basis[basis_i] = *dir;
            basis_i += 1;
            true
        });

        let ref_pt = self.vertices[0];

        for pt in &self.vertices {
            let dpt = *pt - ref_pt;
            let coords = Point2::new(basis[0].dot(&dpt), basis[1].dot(&dpt));
            cache.poly1.push(coords);
        }

        for pt in &other.vertices {
            let dpt = *pt - ref_pt;
            let coords = Point2::new(basis[0].dot(&dpt), basis[1].dot(&dpt));
            cache.poly2.push(coords);
        }

        if cache.poly2.len() > 2 {
            for i in 0..cache.poly1.len() {
                let pt = &cache.poly1[i];

                if utils::point_in_poly2d(pt, &cache.poly2) {
                    let origin = ref_pt + basis[0] * pt.x + basis[1] * pt.y;

                    let n2 = other.normal.as_ref().unwrap().into_inner();
                    let p2 = &other.vertices[0];
                    if let Some(toi2) =
                        query::line_toi_with_plane(p2, &n2, &origin, &normal.into_inner())
                    {
                        let world2 = origin + normal.into_inner() * toi2;
                        let world1 = self.vertices[i];
                        let f2 = other.feature_id;
                        let f1 = self.vertices_id[i];
                        let contact = Contact::new_wo_depth(world1, world2, *normal);

                        if -contact.depth <= prediction.linear() {
                            out.push((contact, f1, f2));
                        }
                    }
                }
            }
        }

        if cache.poly1.len() > 2 {
            for i in 0..cache.poly2.len() {
                let pt = &cache.poly2[i];

                if utils::point_in_poly2d(pt, &cache.poly1) {
                    let origin = ref_pt + basis[0] * pt.x + basis[1] * pt.y;

                    let n1 = self.normal.as_ref().unwrap().into_inner();
                    let p1 = &self.vertices[0];
                    if let Some(toi1) =
                        query::line_toi_with_plane(p1, &n1, &origin, &normal.into_inner())
                    {
                        let world1 = origin + normal.into_inner() * toi1;
                        let world2 = other.vertices[i];
                        let f1 = self.feature_id;
                        let f2 = other.vertices_id[i];
                        let contact = Contact::new_wo_depth(world1, world2, *normal);

                        if -contact.depth <= prediction.linear() {
                            out.push((contact, f1, f2));
                        }
                    }
                }
            }
        }

        let nedges1 = self.nedges();
        let nedges2 = other.nedges();

        for i1 in 0..nedges1 {
            let j1 = (i1 + 1) % cache.poly1.len();
            let seg1 = (&cache.poly1[i1], &cache.poly1[j1]);

            for i2 in 0..nedges2 {
                let j2 = (i2 + 1) % cache.poly2.len();
                let seg2 = (&cache.poly2[i2], &cache.poly2[j2]);

                if let (SegmentPointLocation::OnEdge(e1), SegmentPointLocation::OnEdge(e2)) =
                    query::closest_points_segment_segment_with_locations_nD(seg1, seg2)
                {
                    let original1 = Segment::new(self.vertices[i1], self.vertices[j1]);
                    let original2 = Segment::new(other.vertices[i2], other.vertices[j2]);
                    let world1 = original1.point_at(&SegmentPointLocation::OnEdge(e1));
                    let world2 = original2.point_at(&SegmentPointLocation::OnEdge(e2));
                    let f1 = self.edges_id[i1];
                    let f2 = other.edges_id[i2];
                    let contact = Contact::new_wo_depth(world1, world2, *normal);

                    if -contact.depth <= prediction.linear() {
                        out.push((contact, f1, f2));
                    }
                }
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
                    m1.inverse_transform_unit_vector(self.normal.as_ref().unwrap()),
                ),
            ),
            FeatureId::Edge(..) => {
                let e1 = self.edge(f1).expect("Invalid edge id.");
                if let Some(dir1) = e1.direction() {
                    let local_dir1 = m1.inverse_transform_unit_vector(&dir1);
                    let approx1 = NeighborhoodGeometry::Line(local_dir1);
                    kinematic.set_approx1(f1, local1, approx1)
                } else {
                    return;
                }
            }
            FeatureId::Vertex(..) => kinematic.set_approx1(f1, local1, NeighborhoodGeometry::Point),
            FeatureId::Unknown => return,
        }

        match f2 {
            FeatureId::Face(..) => {
                let approx2 = NeighborhoodGeometry::Plane(
                    m2.inverse_transform_unit_vector(&other.normal.as_ref().unwrap()),
                );
                kinematic.set_approx2(f2, local2, approx2)
            }
            FeatureId::Edge(..) => {
                let e2 = other.edge(f2).expect("Invalid edge id.");
                if let Some(dir2) = e2.direction() {
                    let local_dir2 = m2.inverse_transform_unit_vector(&dir2);
                    let approx2 = NeighborhoodGeometry::Line(local_dir2);
                    kinematic.set_approx2(f2, local2, approx2)
                } else {
                    return;
                }
            }
            FeatureId::Vertex(..) => kinematic.set_approx2(f2, local2, NeighborhoodGeometry::Point),
            FeatureId::Unknown => return,
        }

        let _ = manifold.push(c, kinematic, local1, proc1, proc2);
    }
}
