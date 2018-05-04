use na::{self, Real, Unit};

use shape::{Segment, FeatureId};
use query::Contact;
use math::{Isometry, Point, Vector};

/// Represents a convex polygonal approximation of a face of a solid.
///
/// It is never checked if the vertices actually form a convex polygon.
/// If they do not, results of any geometric query may end up being invalid.
#[derive(Clone, Debug)]
pub struct ConvexPolygonalFeature<N: Real> {
    // FIXME: don't keep all those public.
    /// The vertices of this face.
    pub vertices: Vec<Point<N>>,
    /// The outward normal of the edges if it is a face.
    pub edge_normals: Vec<Vector<N>>,
    /// The normal of this feature if it is an edge.
    pub normal: Option<Unit<Vector<N>>>,
    /// The shape-dependent identifier of this feature.
    pub feature_id: FeatureId,
    /// The shape-dependent indentifier of each vertex of this feature.
    pub vertices_id: Vec<FeatureId>,
    /// The shape-dependent indentifier of each edge of this feature.
    pub edges_id: Vec<FeatureId>,
}

impl<N: Real> ConvexPolygonalFeature<N> {
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
        if let Some(normal) = na::try_normalize(&normal, N::default_epsilon()) {
            self.edge_normals.push(normal)
        } else {
            self.edge_normals.push(na::zero())
        }
    }

    /// Adds an edge normal to this face.
    pub fn push_edge_normal(&mut self, normal: Unit<Vector<N>>) {
        self.edge_normals.push(normal.unwrap())
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
}
