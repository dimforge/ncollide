use std::collections::HashMap;
use std::collections::hash_map::Entry;
use utils::{self, data::SortedPair};

use na::{self, Point2, Point3, Unit};

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
}

#[derive(PartialEq, Debug, Copy, Clone)]
struct Face<V: Vector> {
    vertices: Point3<usize>,
    edges: Point3<usize>,
    normal: Unit<V>,
}

#[derive(PartialEq, Debug, Clone)]
/// The implicit convex hull of a set of points.
pub struct ConvexHull<P: Point> {
    points: Vec<P>,
    vertices: Vec<Vertex>,
    faces: Vec<Face<P::Vector>>,
    edges: Vec<Edge>,
    adj_faces: Vec<usize>,
}

impl<P: Point> ConvexHull<P> {
    #[inline]
    pub fn try_new(points: Vec<P>, indices: &[usize]) -> Option<ConvexHull<P>> {
        let dim = na::dimension::<P::Vector>();

        let mut vertices = Vec::new();
        let mut faces = Vec::new();
        let mut edges = Vec::<Edge>::new();
        let mut adj_faces = Vec::new();

        let empty_vertex = Vertex {
            first_adj_face: 0,
            num_adj_faces: 0,
        };

        vertices.resize(points.len(), empty_vertex);

        //// Euler characteristic.
        let nedges = points.len() + (indices.len() / dim) - 2;
        edges.reserve(nedges);

        if dim == 2 {
            return None;
        } else {
            let mut edge_map = HashMap::<SortedPair<usize>, usize>::new();

            // Initialize vertices
            //// First, find their multiplicities.
            for i in indices {
                vertices[*i].num_adj_faces += 1;
            }

            //// Now, find their starting id.
            let mut total_num_adj_faces = 0;
            for v in &mut vertices {
                v.first_adj_face = total_num_adj_faces;
                total_num_adj_faces += v.num_adj_faces;
            }

            adj_faces.resize(total_num_adj_faces, 0);

            //// Reset the number of adjascent faces.
            //// It will be reinitialized when the face ids will
            //// be added.
            for v in &mut vertices {
                v.num_adj_faces = 0;
            }

            // Initialize faces.
            for vtx in indices.chunks(3) {
                let mut edges_id = Point3::origin();
                let face_id = faces.len();

                for i1 in 0..3 {
                    // Deal with vertices' adjascent faces.
                    let v = &mut vertices[vtx[i1]];
                    adj_faces[v.first_adj_face + v.num_adj_faces] = face_id;
                    v.num_adj_faces += 1;

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
                            })
                        }
                    }
                }

                let vertices = Point3::new(vtx[0], vtx[1], vtx[2]);
                let normal =
                    P::ccw_face_normal(&[&points[vtx[0]], &points[vtx[1]], &points[vtx[2]]])?;
                let face = Face {
                    vertices,
                    edges: edges_id,
                    normal,
                };

                faces.push(face);
            }

            // Check that the Euler characteristic is respected.
            if nedges != edges.len() {
                None
            } else {
                Some(ConvexHull {
                    points,
                    vertices,
                    faces,
                    edges,
                    adj_faces,
                })
            }
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
        let v1 = face.vertices[0];
        let v2 = face.vertices[1];
        let v3 = face.vertices[2];
        out.push(self.points[v1], FeatureId::Vertex(v1));
        out.push(self.points[v2], FeatureId::Vertex(v2));
        out.push(self.points[v3], FeatureId::Vertex(v3));
        out.push_edge_feature_id(FeatureId::Edge(face.edges[0]));
        out.push_edge_feature_id(FeatureId::Edge(face.edges[1]));
        out.push_edge_feature_id(FeatureId::Edge(face.edges[2]));
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
