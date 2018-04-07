use std::collections::HashMap;
use std::collections::hash_map::Entry;
use num::One;
use approx::ApproxEq;

use na::{self, Point2, Point3, Unit, Real};

use utils::{self, data::SortedPair};
use bounding_volume::PolyhedralCone;
use shape::{ConvexPolyface, ConvexPolyhedron, FeatureId, SupportMap};
use math::{Isometry, Point, Vector};

#[derive(PartialEq, Debug, Copy, Clone)]
struct Vertex {
    first_adj_face: usize,
    num_adj_faces: usize,
}

#[derive(PartialEq, Debug, Copy, Clone)]
struct Edge {
    vertices: Point2<usize>,
    faces: Point2<usize>,
    deleted: bool
}

impl Edge {
    fn other_triangle(&self, id: usize) -> usize {
        if id == self.faces[0] {
            self.faces[1]
        } else {
            self.faces[0]
        }
    }
}

#[derive(PartialEq, Debug, Copy, Clone)]
struct Face<V: Vector> {
    first_vertex_or_edge: usize,
    num_vertices_or_edges: usize,
    normal: Unit<V>,
}


#[derive(PartialEq, Debug, Copy, Clone)]
struct Triangle<V: Vector> {
    vertices: Point3<usize>,
    edges: Point3<usize>,
    normal: Unit<V>,
    parent_face: Option<usize>,
}

impl<V: Vector> Triangle<V> {
    fn next_edge_id(&self, id: usize) -> usize {
        for i in 0 .. 3 {
            if self.edges[i] == id {
                return (i + 1) % 3;
            }
        }

        unreachable!()
    }
}

#[derive(PartialEq, Debug, Clone)]
/// A convex polyhedron without degenerate faces.
pub struct ConvexHull<P: Point> {
    points: Vec<P>,
    vertices: Vec<Vertex>,
    faces: Vec<Face<P::Vector>>,
    edges: Vec<Edge>,
    // Faces adjascent to a vertex.
    adj_faces: Vec<usize>,
    // Edges adjascent to a face.
    adj_edges: Vec<usize>,
    // Vertices adjascent to a face.
    adj_vertices: Vec<usize>,
}

impl<P: Point> ConvexHull<P> {
    #[inline]
    pub fn try_new(points: Vec<P>, indices: &[usize]) -> Option<ConvexHull<P>> {
        let eps = P::Real::default_epsilon().sqrt();
        let dim = na::dimension::<P::Vector>();
        
        if dim != 3 {
            return None;
        }

        let mut vertices = Vec::new();
        let mut edges = Vec::<Edge>::new();
        let mut faces = Vec::<Face<P::Vector>>::new();
        let mut triangles = Vec::new();
        let mut edge_map = HashMap::<SortedPair<usize>, usize>::new();

        let mut adj_faces = Vec::new();
        let mut adj_edges = Vec::new();
        let mut adj_vertices = Vec::new();

        //// Euler characteristic.
        let nedges = points.len() + (indices.len() / dim) - 2;
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
                        edges.push(Edge {
                            vertices: Point2::new(vtx[i1], vtx[i2]),
                            faces: Point2::new(face_id, 0),
                            deleted: false,
                        })
                    }
                }
            }

            let vertices = Point3::new(vtx[0], vtx[1], vtx[2]);
            let normal =
                P::ccw_face_normal(&[&points[vtx[0]], &points[vtx[1]], &points[vtx[2]]])?;
            let triangle = Triangle {
                vertices,
                edges: edges_id,
                normal,
                parent_face: None
            };

            triangles.push(triangle);
        }

        // Find edges that must be deleted.
        let mut num_valid_edges = 0;

        for e in &mut edges {
            let n1 = triangles[e.faces[0]].normal;
            let n2 = triangles[e.faces[1]].normal;
            if na::dot(&*n1, &*n2) > P::Real::one() - eps {
                e.deleted = true;
            } else {
                num_valid_edges += 1;
            }
        }

        /*
         * Extract faces by following  contours.
         */
        for i in 0 .. triangles.len() {
            if triangles[i].parent_face.is_none() {
                for j1 in 0..3 {
                    if !edges[triangles[i].edges[j1]].deleted {
                        // Create a new face, setup its first edge/vertex and construct it.
                        let new_face_id = faces.len();
                        let mut new_face = Face {
                            first_vertex_or_edge: adj_edges.len(),
                            num_vertices_or_edges: 1,
                            normal: triangles[i].normal,
                        };

                        adj_edges.push(triangles[i].edges[j1]);
                        adj_vertices.push(triangles[i].vertices[j1]);


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
                                adj_edges.push(curr_edge);
                                adj_vertices.push(curr_vertex);
                                new_face.num_vertices_or_edges += 1;

                                curr_edge_id = (curr_edge_id + 1) % 3;
                            } else {
                                // Find adjascent edge on the next triange.
                                curr_triangle = edges[curr_edge].other_triangle(curr_triangle);
                                curr_edge_id = triangles[curr_triangle].next_edge_id(curr_edge);
                                assert!(triangles[curr_triangle].vertices[curr_edge_id] == curr_vertex);
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
            first_adj_face: 0,
            num_adj_faces: 0,
        };

        vertices.resize(points.len(), empty_vertex);

        // First, find their multiplicities.
        for face in &faces {
            let first_vid = face.first_vertex_or_edge;
            let last_vid = face.first_vertex_or_edge + face.num_vertices_or_edges;

            for i in &adj_vertices[first_vid..last_vid] {
                vertices[*i].num_adj_faces += 1;
            }
        }

        // Now, find their starting id.
        let mut total_num_adj_faces = 0;
        for v in &mut vertices {
            v.first_adj_face = total_num_adj_faces;
            total_num_adj_faces += v.num_adj_faces;
        }
        adj_faces.resize(total_num_adj_faces, 0);

        // Reset the number of adjascent faces.
        // It will be set againt to the right value as
        // the adjascent face list is filled.
        for v in &mut vertices {
            v.num_adj_faces = 0;
        }

        for face_id in 0..faces.len() {
            let face = &faces[face_id];
            let first_vid = face.first_vertex_or_edge;
            let last_vid = face.first_vertex_or_edge + face.num_vertices_or_edges;

            for i in &adj_vertices[first_vid..last_vid] {
                let v = &mut vertices[*i];
                adj_faces[v.first_adj_face + v.num_adj_faces] = face_id;
                v.num_adj_faces += 1;
            }
        }

        let mut num_valid_vertices = 0;

        for v in &vertices {
            if v.num_adj_faces != 0 {
                num_valid_vertices += 1;
            }
        }

        // Check that the Euler characteristic is respected.
        if num_valid_vertices + faces.len() - num_valid_edges != 2 {
            None
        } else {
            Some(ConvexHull {
                points,
                vertices,
                faces,
                edges,
                adj_faces,
                adj_edges,
                adj_vertices
            })
        }
    }

    #[inline]
    pub fn points(&self) -> &[P] {
        &self.points[..]
    }
}

impl<P: Point, M: Isometry<P>> SupportMap<P, M> for ConvexHull<P> {
    #[inline]
    fn support_point(&self, m: &M, dir: &P::Vector) -> P {
        let local_dir = m.inverse_rotate_vector(dir);
        let best_pt = utils::point_cloud_support_point(&local_dir, self.points());

        m.transform_point(&best_pt)
    }
}

impl<P: Point, M: Isometry<P>> ConvexPolyhedron<P, M> for ConvexHull<P> {
    fn vertex(&self, id: FeatureId) -> P {
        self.points[id.unwrap_vertex()]
    }

    fn edge(&self, id: FeatureId) -> (P, P, FeatureId, FeatureId) {
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

    fn face(&self, id: FeatureId, out: &mut ConvexPolyface<P>) {
        out.clear();

        let face = &self.faces[id.unwrap_face()];
        let first_vertex = face.first_vertex_or_edge;
        let last_vertex = face.first_vertex_or_edge + face.num_vertices_or_edges;

        for i in first_vertex..last_vertex {
            let vid = self.adj_vertices[i];
            let eid = self.adj_edges[i];
            out.push(self.points[vid], FeatureId::Vertex(vid));
            out.push_edge_feature_id(FeatureId::Edge(eid));
        }

        out.set_normal(face.normal);
        out.set_feature_id(id);
        out.recompute_edge_normals_3d();
    }

    fn normal_cone(&self, feature: FeatureId) -> PolyhedralCone<P> {
        let mut polycone = PolyhedralCone::new();

        match feature {
            FeatureId::Face(id) => polycone.add_generator(self.faces[id].normal),
            FeatureId::Edge(id) => {
                let edge = &self.edges[id];
                polycone.add_generator(self.faces[edge.faces[0]].normal);
                polycone.add_generator(self.faces[edge.faces[1]].normal);
            }
            FeatureId::Vertex(id) => {
                let vertex = &self.vertices[id];
                let first = vertex.first_adj_face;
                let last = vertex.first_adj_face + vertex.num_adj_faces;

                for face in &self.adj_faces[first..last] {
                    polycone.add_generator(self.faces[*face].normal)
                }
            }
            FeatureId::Unknown => {}
        }

        polycone
    }

    fn support_face_toward(&self, m: &M, dir: &Unit<P::Vector>, out: &mut ConvexPolyface<P>) {
        let ls_dir = m.inverse_rotate_vector(dir);
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

        ConvexPolyhedron::<P, M>::face(self, FeatureId::Face(best_face), out);
        out.transform_by(m);
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
}
