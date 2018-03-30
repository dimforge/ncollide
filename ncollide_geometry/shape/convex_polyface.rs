use approx::ApproxEq;
use na::{self, Id, Real, Unit};

use math::{Isometry, Point};
use utils::{self, IdAllocator};
use shape::{Segment, SegmentPointLocation, SupportMap};
use query::{Contact, ContactManifold, ContactPrediction, PointQueryWithLocation};
use query::closest_points_internal;

#[derive(Copy, Clone, Debug, Hash, PartialEq, Eq)]
pub enum FeatureId {
    Vertex { subshape: usize, id: usize },
    Edge { subshape: usize, id: usize },
    Face { subshape: usize, id: usize },
    Unknown,
}

impl FeatureId {
    #[inline]
    pub fn vertex(subshape: usize, id: usize) -> FeatureId {
        FeatureId::Vertex { subshape, id }
    }

    #[inline]
    pub fn edge(subshape: usize, id: usize) -> FeatureId {
        FeatureId::Edge { subshape, id }
    }

    #[inline]
    pub fn face(subshape: usize, id: usize) -> FeatureId {
        FeatureId::Face { subshape, id }
    }

    #[inline]
    pub fn subshape_id(&self) -> usize {
        match *self {
            FeatureId::Vertex { subshape, .. } => subshape,
            FeatureId::Edge { subshape, .. } => subshape,
            FeatureId::Face { subshape, .. } => subshape,
            FeatureId::Unknown => 0,
        }
    }

    #[inline]
    pub fn vertex_id(&self) -> Option<usize> {
        match *self {
            FeatureId::Vertex { id, .. } => Some(id),
            _ => None,
        }
    }

    #[inline]
    pub fn edge_id(&self) -> Option<usize> {
        match *self {
            FeatureId::Edge { id, .. } => Some(id),
            _ => None,
        }
    }

    #[inline]
    pub fn face_id(&self) -> Option<usize> {
        match *self {
            FeatureId::Face { id, .. } => Some(id),
            _ => None,
        }
    }
}

/// Represents a convex polygonal approximation of a face of a solid.
///
/// It is never checked if the vertices actually form a convex polygon.
/// If they do not, results of any geometric query may end up being invalid.
#[derive(Clone, Debug)]
pub struct ConvexPolyface<P: Point> {
    // FIXME: don't keep all those public.
    /// The vertices of the polyhedral face.
    pub vertices: Vec<P>,
    pub edge_normals: Vec<P::Vector>,
    pub normal: Option<Unit<P::Vector>>,
    pub feature_id: FeatureId,
    pub vertices_id: Vec<FeatureId>,
    pub edges_id: Vec<FeatureId>,
}

impl<P: Point> ConvexPolyface<P> {
    /// Creates a new empty convex polygonal faces.
    pub fn new() -> Self {
        ConvexPolyface {
            vertices: Vec::new(),
            edge_normals: Vec::new(),
            normal: None,
            feature_id: FeatureId::Unknown,
            vertices_id: Vec::new(),
            edges_id: Vec::new(),
        }
    }

    /// Removes all the vertices of this face.
    pub fn clear(&mut self) {
        self.vertices.clear();
        self.edge_normals.clear();
        self.vertices_id.clear();
        self.edges_id.clear();
        self.normal = None;
        self.feature_id = FeatureId::Unknown;
    }

    pub fn reduce_to_feature(&mut self, id: FeatureId) {
        if id == self.feature_id {
            return;
        }

        self.feature_id = id;
        self.normal = None;
        self.edge_normals.clear();

        for i in 0..self.vertices.len() {
            if self.vertices_id[i] == id {
                self.vertices[0] = self.vertices[i];
                self.vertices_id[0] = self.vertices_id[i];
                self.vertices.resize(1, P::origin());
                self.vertices_id.resize(1, FeatureId::Unknown);
                return;
            }

            if self.edges_id[i] == id {
                self.edges_id[0] = id;

                if i == self.vertices.len() - 1 {
                    self.vertices[1] = self.vertices[i];
                    self.vertices_id[1] = self.vertices_id[i];
                } else {
                    self.vertices[0] = self.vertices[i + 0];
                    self.vertices[1] = self.vertices[i + 1];
                    self.vertices_id[0] = self.vertices_id[i + 0];
                    self.vertices_id[1] = self.vertices_id[i + 1];
                }
                self.vertices.resize(2, P::origin());
                self.vertices_id.resize(2, FeatureId::Unknown);
                return;
            }
        }

        // Unknown feature: reduce to nothing.
        self.clear();
    }

    pub fn transform_by<M: Isometry<P>>(&mut self, m: &M) {
        for p in &mut self.vertices {
            *p = m.transform_point(p);
        }

        for n in &mut self.edge_normals {
            let new_n = m.transform_vector(n);
            *n = new_n;
        }

        if let Some(ref mut n) = self.normal {
            let new_n = m.transform_vector(n.as_ref());
            *n = Unit::new_unchecked(new_n);
        }
    }

    /// Adds a vertex to this face.
    ///
    /// It is not checked whether `pt` breaks the convexity of the polyhedral face.
    pub fn push(&mut self, pt: P, id: FeatureId) {
        self.vertices.push(pt);
        self.vertices_id.push(id);
    }

    /// The number of vertices of this face.
    pub fn nvertices(&self) -> usize {
        self.vertices.len()
    }

    /// The vertices of this convex polygonal face.
    pub fn vertices(&self) -> &[P] {
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
    pub fn edge(&self, edge_id: FeatureId) -> Option<Segment<P>> {
        for i1 in 0..self.vertices.len() {
            if self.edges_id[i1] == edge_id {
                let i2 = (i1 + 1) % self.vertices.len();
                return Some(Segment::new(self.vertices[i1], self.vertices[i2]));
            }
        }

        None
    }

    /// Adds a scaled edge normal to this face.
    pub fn push_scaled_edge_normal(&mut self, normal: P::Vector) {
        if let Some(normal) = na::try_normalize(&normal, P::Real::default_epsilon()) {
            self.edge_normals.push(normal)
        } else {
            self.edge_normals.push(na::zero())
        }
    }

    /// Adds an edge normal to this face.
    pub fn push_edge_normal(&mut self, normal: Unit<P::Vector>) {
        self.edge_normals.push(normal.unwrap())
    }

    /// Automatically recomputes the scaled edge normals (3D only).
    ///
    /// Panics if the ambient space is not 3D.
    pub fn recompute_edge_normals_3d(&mut self) {
        self.edge_normals.clear();

        for i1 in 0..self.vertices.len() {
            let i2 = (i1 + 1) % self.vertices.len();
            let dpt = self.vertices[i2] - self.vertices[i1];
            let scaled_normal = utils::cross3(
                &dpt,
                self.normal
                    .as_ref()
                    .expect("The face normal must be set before computing edge normals."),
            );
            self.push_scaled_edge_normal(scaled_normal)
        }
    }

    pub fn project_point(&self, pt: &P) -> Option<Contact<P>> {
        if let Some(n) = self.normal {
            let dpt = *pt - self.vertices[0];
            let dist = na::dot(n.as_ref(), &dpt);
            let proj = *pt + (-n.unwrap() * dist);

            for i in 0..self.edge_normals.len() {
                let dpt = proj - self.vertices[i];

                if na::dot(&dpt, &self.edge_normals[i]) > na::zero() {
                    return None;
                }
            }

            Some(Contact::new(proj, *pt, n, -dist))
        } else {
            None
        }
    }

    /// Sets the outward normal of this convex polygonal face.
    pub fn set_normal(&mut self, normal: Unit<P::Vector>) {
        self.normal = Some(normal)
    }

    pub fn push_edge_feature_id(&mut self, id: FeatureId) {
        self.edges_id.push(id)
    }

    pub fn set_feature_id(&mut self, id: FeatureId) {
        self.feature_id = id
    }
}
