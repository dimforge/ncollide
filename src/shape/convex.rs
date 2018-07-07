use smallvec::SmallVec;
use std::collections::hash_map::Entry;
use std::collections::HashMap;
use std::f64;

use na::{self, Point2, Point3, Real, Unit};

use bounding_volume::PolyhedralCone;
use math::{Isometry, Point, Vector};
use shape::{ConvexPolygonalFeature, ConvexPolyhedron, FeatureId, SupportMap};
use transformation;
use utils::{self, IsometryOps, SortedPair};

#[derive(PartialEq, Debug, Copy, Clone)]
struct Vertex {
    first_adj_face_or_edge: usize,
    num_adj_faces_or_edge: usize,
}

#[derive(PartialEq, Debug, Copy, Clone)]
struct Edge<N: Real> {
    vertices: Point2<usize>,
    faces: Point2<usize>,
    dir: Unit<Vector<N>>,
    deleted: bool,
}

impl<N: Real> Edge<N> {
    fn other_triangle(&self, id: usize) -> usize {
        if id == self.faces[0] {
            self.faces[1]
        } else {
            self.faces[0]
        }
    }
}

#[derive(PartialEq, Debug, Copy, Clone)]
struct Face<N: Real> {
    first_vertex_or_edge: usize,
    num_vertices_or_edges: usize,
    normal: Unit<Vector<N>>,
}

#[derive(PartialEq, Debug, Copy, Clone)]
struct Triangle<N: Real> {
    vertices: Point3<usize>,
    edges: Point3<usize>,
    normal: Unit<Vector<N>>,
    parent_face: Option<usize>,
}

impl<N: Real> Triangle<N> {
    fn next_edge_id(&self, id: usize) -> usize {
        for i in 0..3 {
            if self.edges[i] == id {
                return (i + 1) % 3;
            }
        }

        unreachable!()
    }
}

#[derive(PartialEq, Debug, Clone)]
/// A convex polyhedron without degenerate faces.
pub struct ConvexHull<N: Real> {
    points: Vec<Point<N>>,
    vertices: Vec<Vertex>,
    faces: Vec<Face<N>>,
    edges: Vec<Edge<N>>,
    // Faces adjascent to a vertex.
    faces_adj_to_vertex: Vec<usize>,
    // Edges adjascent to a vertex.
    edges_adj_to_vertex: Vec<usize>,
    // Edges adjascent to a face.
    edges_adj_to_face: Vec<usize>,
    // Vertices adjascent to a face.
    vertices_adj_to_face: Vec<usize>,
}

impl<N: Real> ConvexHull<N> {
    /// Creates a new 2D convex polyhedron from an arbitrary set of points.
    ///
    /// This explicitly computes the convex hull of the given set of points. Use
    /// Returns `None` if the convex hull computation failed.
    pub fn try_from_points(points: &[Point<N>]) -> Option<ConvexHull<N>> {
        let hull = transformation::convex_hull(points);
        let indices: Vec<usize> = hull.flat_indices()
            .into_iter()
            .map(|i| i as usize)
            .collect();

        Self::try_new(hull.coords, &indices)
    }
    /// Attempts to create a new solid assumed to be convex from the set of points and indices.
    ///
    /// The given points and index informations are assumed to describe a convex polyhedron.
    /// It it is not, weird results may be produced.
    ///
    /// # Return
    ///
    /// Retruns `None` if:
    ///
    ///   1. The given solid does not satisfy the euler characteristic.
    ///   2. The given solid contains degenerate edges/triangles.
    pub fn try_new(points: Vec<Point<N>>, indices: &[usize]) -> Option<ConvexHull<N>> {
        let eps = N::default_epsilon().sqrt();

        let mut vertices = Vec::new();
        let mut edges = Vec::<Edge<N>>::new();
        let mut faces = Vec::<Face<N>>::new();
        let mut triangles = Vec::new();
        let mut edge_map = HashMap::<SortedPair<usize>, usize>::new();

        let mut faces_adj_to_vertex = Vec::new();
        let mut edges_adj_to_vertex = Vec::new();
        let mut edges_adj_to_face = Vec::new();
        let mut vertices_adj_to_face = Vec::new();

        //// Euler characteristic.
        let nedges = points.len() + (indices.len() / 3) - 2;
        edges.reserve(nedges);

        /*
         *  Initialize triangles and edges adjascency information.
         */
        for vtx in indices.chunks(3) {
            let mut edges_id = Point3::origin();
            let face_id = triangles.len();

            for i1 in 0..3 {
                // Deal with edges.
                let i2 = (i1 + 1) % 3;
                let key = SortedPair::new(vtx[i1], vtx[i2]);

                match edge_map.entry(key) {
                    Entry::Occupied(e) => {
                        edges_id[i1] = *e.get();
                        edges[*e.get()].faces[1] = face_id
                    }
                    Entry::Vacant(e) => {
                        edges_id[i1] = *e.insert(edges.len());

                        if let Some(dir) =
                            Unit::try_new(points[vtx[i2]] - points[vtx[i1]], N::default_epsilon())
                        {
                            edges.push(Edge {
                                vertices: Point2::new(vtx[i1], vtx[i2]),
                                faces: Point2::new(face_id, 0),
                                dir,
                                deleted: false,
                            })
                        } else {
                            return None;
                        }
                    }
                }
            }

            let vertices = Point3::new(vtx[0], vtx[1], vtx[2]);
            let normal =
                utils::ccw_face_normal([&points[vtx[0]], &points[vtx[1]], &points[vtx[2]]])?;
            let triangle = Triangle {
                vertices,
                edges: edges_id,
                normal,
                parent_face: None,
            };

            triangles.push(triangle);
        }

        // Find edges that must be deleted.
        let mut num_valid_edges = 0;

        for e in &mut edges {
            let n1 = triangles[e.faces[0]].normal;
            let n2 = triangles[e.faces[1]].normal;
            if na::dot(&*n1, &*n2) > N::one() - eps {
                e.deleted = true;
            } else {
                num_valid_edges += 1;
            }
        }

        /*
         * Extract faces by following  contours.
         */
        for i in 0..triangles.len() {
            if triangles[i].parent_face.is_none() {
                for j1 in 0..3 {
                    if !edges[triangles[i].edges[j1]].deleted {
                        // Create a new face, setup its first edge/vertex and construct it.
                        let new_face_id = faces.len();
                        let mut new_face = Face {
                            first_vertex_or_edge: edges_adj_to_face.len(),
                            num_vertices_or_edges: 1,
                            normal: triangles[i].normal,
                        };

                        edges_adj_to_face.push(triangles[i].edges[j1]);
                        vertices_adj_to_face.push(triangles[i].vertices[j1]);

                        let j2 = (j1 + 1) % 3;
                        let start_vertex = triangles[i].vertices[j1];

                        // NOTE: variables ending with _id are identifier on the
                        // fields of a triangle. Other variables are identifier on
                        // the triangles/edges/vertices arrays.
                        let mut curr_triangle = i;
                        let mut curr_edge_id = j2;

                        while triangles[curr_triangle].vertices[curr_edge_id] != start_vertex {
                            let curr_edge = triangles[curr_triangle].edges[curr_edge_id];
                            let curr_vertex = triangles[curr_triangle].vertices[curr_edge_id];
                            triangles[curr_triangle].parent_face = Some(new_face_id);

                            if !edges[curr_edge].deleted {
                                edges_adj_to_face.push(curr_edge);
                                vertices_adj_to_face.push(curr_vertex);
                                new_face.num_vertices_or_edges += 1;

                                curr_edge_id = (curr_edge_id + 1) % 3;
                            } else {
                                // Find adjascent edge on the next triange.
                                curr_triangle = edges[curr_edge].other_triangle(curr_triangle);
                                curr_edge_id = triangles[curr_triangle].next_edge_id(curr_edge);
                                assert!(
                                    triangles[curr_triangle].vertices[curr_edge_id] == curr_vertex
                                );
                            }
                        }

                        faces.push(new_face);
                        break;
                    }
                }
            }
        }

        // Update face ids inside edges so that they point to the faces instead of the triangles.
        for e in &mut edges {
            if let Some(fid) = triangles[e.faces[0]].parent_face {
                e.faces[0] = fid;
            }

            if let Some(fid) = triangles[e.faces[1]].parent_face {
                e.faces[1] = fid;
            }
        }

        /*
         * Initialize vertices
         */
        let empty_vertex = Vertex {
            first_adj_face_or_edge: 0,
            num_adj_faces_or_edge: 0,
        };

        vertices.resize(points.len(), empty_vertex);

        // First, find their multiplicities.
        for face in &faces {
            let first_vid = face.first_vertex_or_edge;
            let last_vid = face.first_vertex_or_edge + face.num_vertices_or_edges;

            for i in &vertices_adj_to_face[first_vid..last_vid] {
                vertices[*i].num_adj_faces_or_edge += 1;
            }
        }

        // Now, find their starting id.
        let mut total_num_adj_faces = 0;
        for v in &mut vertices {
            v.first_adj_face_or_edge = total_num_adj_faces;
            total_num_adj_faces += v.num_adj_faces_or_edge;
        }
        faces_adj_to_vertex.resize(total_num_adj_faces, 0);
        edges_adj_to_vertex.resize(total_num_adj_faces, 0);

        // Reset the number of adjascent faces.
        // It will be set againt to the right value as
        // the adjascent face list is filled.
        for v in &mut vertices {
            v.num_adj_faces_or_edge = 0;
        }

        for face_id in 0..faces.len() {
            let face = &faces[face_id];
            let first_vid = face.first_vertex_or_edge;
            let last_vid = face.first_vertex_or_edge + face.num_vertices_or_edges;

            for vid in first_vid..last_vid {
                let v = &mut vertices[vertices_adj_to_face[vid]];
                faces_adj_to_vertex[v.first_adj_face_or_edge + v.num_adj_faces_or_edge] = face_id;
                edges_adj_to_vertex[v.first_adj_face_or_edge + v.num_adj_faces_or_edge] =
                    edges_adj_to_face[vid];
                v.num_adj_faces_or_edge += 1;
            }
        }

        let mut num_valid_vertices = 0;

        for v in &vertices {
            if v.num_adj_faces_or_edge != 0 {
                num_valid_vertices += 1;
            }
        }

        // Check that the Euler characteristic is respected.
        if num_valid_vertices + faces.len() - num_valid_edges != 2 {
            None
        } else {
            let res = ConvexHull {
                points,
                vertices,
                faces,
                edges,
                faces_adj_to_vertex,
                edges_adj_to_vertex,
                edges_adj_to_face,
                vertices_adj_to_face,
            };

            // FIXME: for debug.
            // res.check_geometry();

            Some(res)
        }
    }

    /// Verify if this convex polyhedron is actually convex.
    #[inline]
    pub fn check_geometry(&self) {
        for face in &self.faces {
            let p0 = self.points[self.vertices_adj_to_face[face.first_vertex_or_edge]];

            for v in &self.points {
                assert!((v - p0).dot(face.normal.as_ref()) <= N::default_epsilon());
            }
        }
    }

    /// The set of vertices of this convex polyhedron.
    #[inline]
    pub fn points(&self) -> &[Point<N>] {
        &self.points[..]
    }
}

impl<N: Real> SupportMap<N> for ConvexHull<N> {
    #[inline]
    fn local_support_point(&self, dir: &Vector<N>) -> Point<N> {
        utils::point_cloud_support_point(dir, self.points())
    }
}

impl<N: Real> ConvexPolyhedron<N> for ConvexHull<N> {
    fn vertex(&self, id: FeatureId) -> Point<N> {
        self.points[id.unwrap_vertex()]
    }

    fn edge(&self, id: FeatureId) -> (Point<N>, Point<N>, FeatureId, FeatureId) {
        let edge = &self.edges[id.unwrap_edge()];
        let v1 = edge.vertices[0];
        let v2 = edge.vertices[1];

        (
            self.points[v1],
            self.points[v2],
            FeatureId::Vertex(v1),
            FeatureId::Vertex(v2),
        )
    }

    fn face(&self, id: FeatureId, out: &mut ConvexPolygonalFeature<N>) {
        out.clear();

        let face = &self.faces[id.unwrap_face()];
        let first_vertex = face.first_vertex_or_edge;
        let last_vertex = face.first_vertex_or_edge + face.num_vertices_or_edges;

        for i in first_vertex..last_vertex {
            let vid = self.vertices_adj_to_face[i];
            let eid = self.edges_adj_to_face[i];
            out.push(self.points[vid], FeatureId::Vertex(vid));
            out.push_edge_feature_id(FeatureId::Edge(eid));
        }

        out.set_normal(face.normal);
        out.set_feature_id(id);
        out.recompute_edge_normals();
    }

    fn normal_cone(&self, feature: FeatureId) -> PolyhedralCone<N> {
        let mut generators = SmallVec::new();

        match feature {
            FeatureId::Face(id) => return PolyhedralCone::HalfLine(self.faces[id].normal),
            FeatureId::Edge(id) => {
                let edge = &self.edges[id];
                generators.push(self.faces[edge.faces[0]].normal);
                generators.push(self.faces[edge.faces[1]].normal);
            }
            FeatureId::Vertex(id) => {
                let vertex = &self.vertices[id];
                let first = vertex.first_adj_face_or_edge;
                let last = vertex.first_adj_face_or_edge + vertex.num_adj_faces_or_edge;

                for face in &self.faces_adj_to_vertex[first..last] {
                    generators.push(self.faces[*face].normal)
                }
            }
            FeatureId::Unknown => {}
        }

        PolyhedralCone::Span(generators)
    }

    fn support_face_toward(
        &self,
        m: &Isometry<N>,
        dir: &Unit<Vector<N>>,
        out: &mut ConvexPolygonalFeature<N>,
    ) {
        let ls_dir = m.inverse_transform_vector(dir);
        let mut best_face = 0;
        let mut max_dot = na::dot(&*self.faces[0].normal, &ls_dir);

        for i in 1..self.faces.len() {
            let face = &self.faces[i];
            let dot = na::dot(&*face.normal, &ls_dir);

            if dot > max_dot {
                max_dot = dot;
                best_face = i;
            }
        }

        self.face(FeatureId::Face(best_face), out);
        out.transform_by(m);
    }

    fn support_feature_toward(
        &self,
        transform: &Isometry<N>,
        dir: &Unit<Vector<N>>,
        _angle: N,
        out: &mut ConvexPolygonalFeature<N>,
    ) {
        out.clear();
        // FIXME: actualy find the support feature.
        self.support_face_toward(transform, dir, out)
    }

    fn support_feature_id_toward(&self, local_dir: &Unit<Vector<N>>) -> FeatureId {
        let eps: N = na::convert(f64::consts::PI / 180.0);
        let (seps, ceps) = eps.sin_cos();
        let support_pt_id = utils::point_cloud_support_point_id(local_dir.as_ref(), &self.points);
        let vertex = &self.vertices[support_pt_id];

        // Check faces.
        for i in 0..vertex.num_adj_faces_or_edge {
            let face_id = self.faces_adj_to_vertex[vertex.first_adj_face_or_edge + i];
            let face = &self.faces[face_id];

            if na::dot(face.normal.as_ref(), local_dir.as_ref()) >= ceps {
                return FeatureId::Face(face_id);
            }
        }

        // Check edges.
        for i in 0..vertex.num_adj_faces_or_edge {
            let edge_id = self.edges_adj_to_vertex[vertex.first_adj_face_or_edge + i];
            let edge = &self.edges[edge_id];

            if na::dot(edge.dir.as_ref(), local_dir.as_ref()).abs() <= seps {
                return FeatureId::Edge(edge_id);
            }
        }

        // The vertex is the support feature.
        FeatureId::Vertex(support_pt_id)
    }
}
