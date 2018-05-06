use na::{Real, Unit};

use math::{Isometry, Point, Vector};
use query::Contact;
use shape::FeatureId;

/// A feature (face or vertex) of a 2D convex polygon.
#[derive(Clone, Debug)]
pub struct ConvexPolygonalFeature<N: Real> {
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

impl<N: Real> ConvexPolygonalFeature<N> {
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
                let proj = *pt + (-n.unwrap() * dist);

                Some(Contact::new(proj, *pt, n, -dist))
            }
        } else {
            None
        }
    }
}
