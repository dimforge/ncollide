//! 2d line strip, 3d triangle mesh, and nd subsimplex mesh.

use crate::bounding_volume::{self, BoundingVolume, AABB};
use crate::math::{Isometry, Point, Vector, DIM};
use crate::partitioning::{BVHImpl, BVT};
use crate::procedural;
use crate::query::{
    Contact, ContactKinematic, ContactPrediction, ContactPreprocessor, LocalShapeApproximation,
    NeighborhoodGeometry,
};
use crate::shape::{
    CompositeShape, DeformableShape, DeformationsType, FeatureId, Segment, Shape, Triangle,
};
use crate::utils::{DeterministicState, IsometryOps};
use na::{self, Id, Point2, Point3, RealField, Unit};
use std::collections::{hash_map::Entry, HashMap};
use std::iter;
use std::ops::Range;
use std::slice;

#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Clone)]
struct DeformationInfos<N: RealField> {
    margin: N,
    curr_timestamp: usize,
    timestamps: Vec<usize>,
    ref_vertices: Vec<Point<N>>,
    tri_to_update: Vec<usize>,
}

#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Clone)]
/// Description of a face adjascent to an edge.
pub struct FaceAdjacentToEdge {
    /// Index of the face.
    pub face_id: usize,
    /// Index of the edge the edge is adjascent to.
    pub edge_id: usize,
}

impl FaceAdjacentToEdge {
    fn new(face_id: usize, edge_id: usize) -> Self {
        FaceAdjacentToEdge { face_id, edge_id }
    }
}

#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Clone)]
/// A face of a triangle mesh.
pub struct TriMeshFace<N: RealField> {
    /// Indices of the vertices of this face.
    pub indices: Point3<usize>,
    /// Indices of the edges of this face.
    pub edges: Point3<usize>,
    bvt_leaf: usize,
    /// The normal of this face if it is not degenerate.
    pub normal: Option<Unit<Vector<N>>>,
    /// Outward edge normals on the face's plane.
    pub side_normals: Option<[Unit<Vector<N>>; 3]>,
}

#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Clone)]
/// An edge of a triangle mesh.
pub struct TriMeshEdge {
    /// The indices of this edge.
    pub indices: Point2<usize>,
    /// The faces adjascent to this edge.
    pub adj_faces: (FaceAdjacentToEdge, FaceAdjacentToEdge),
}

#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Clone)]
/// A vertex of a triangle mesh.
pub struct TriMeshVertex {
    /// Indirect indices of this vertex adjacent faces.
    pub adj_faces: Range<usize>,
    /// Indirect indices of this vertex adjacent vertices.
    pub adj_vertices: Range<usize>,
}

/// A 3d triangle mesh.
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
#[derive(Clone)]
pub struct TriMesh<N: RealField> {
    bvt: BVT<usize, AABB<N>>,
    uvs: Option<Vec<Point2<N>>>,
    points: Vec<Point<N>>,
    vertices: Vec<TriMeshVertex>,
    edges: Vec<TriMeshEdge>,
    faces: Vec<TriMeshFace<N>>,
    adj_face_list: Vec<usize>,
    adj_vertex_list: Vec<usize>,
    deformations: DeformationInfos<N>,
    oriented: bool,
}

impl<N: RealField> TriMesh<N> {
    /// Builds a new mesh.
    pub fn new(
        points: Vec<Point<N>>,
        indices: Vec<Point3<usize>>,
        uvs: Option<Vec<Point2<N>>>,
    ) -> TriMesh<N> {
        let mut leaves = Vec::with_capacity(indices.len());
        let mut vertices: Vec<TriMeshVertex> = iter::repeat(TriMeshVertex {
            adj_faces: 0..0,
            adj_vertices: 0..0,
        })
        .take(points.len())
        .collect();
        let mut faces = Vec::with_capacity(indices.len());

        let edges = Self::edges_list(&indices);
        let adj_face_list = Self::adj_face_list(&indices, &mut vertices);
        let adj_vertex_list = Self::adj_vertex_list(&edges, &mut vertices);

        {
            let is = &*indices;

            for (i, is) in is.iter().enumerate() {
                let triangle = Triangle::new(points[is.x], points[is.y], points[is.z]);
                let normal = triangle.normal();
                let side_normals = normal.map(|n| {
                    [
                        Unit::new_normalize((triangle.b() - triangle.a()).cross(&n)),
                        Unit::new_normalize((triangle.c() - triangle.b()).cross(&n)),
                        Unit::new_normalize((triangle.a() - triangle.c()).cross(&n)),
                    ]
                });

                let bv = triangle.local_aabb();
                leaves.push((i, bv.clone()));
                faces.push(TriMeshFace {
                    indices: *is,
                    edges: Point3::origin(), // Will be set later.
                    bvt_leaf: 0,             // Will be set later.
                    normal,
                    side_normals,
                })
            }
        }

        let bvt = BVT::new_balanced(leaves);

        // Set face.bvt_leaf
        for (i, leaf) in bvt.leaves().iter().enumerate() {
            faces[*leaf.data()].bvt_leaf = i;
        }

        // Set face.edges
        for (i, e) in edges.iter().enumerate() {
            let fid1 = e.adj_faces.0.face_id;
            let fid2 = e.adj_faces.1.face_id;

            for k1 in 0..3 {
                let k2 = (k1 + 1) % 3;

                if (faces[fid1].indices[k1] == e.indices.x
                    && faces[fid1].indices[k2] == e.indices.y)
                    || (faces[fid1].indices[k1] == e.indices.y
                        && faces[fid1].indices[k2] == e.indices.x)
                {
                    faces[fid1].edges[k1] = i;
                }

                if (faces[fid2].indices[k1] == e.indices.x
                    && faces[fid2].indices[k2] == e.indices.y)
                    || (faces[fid2].indices[k1] == e.indices.y
                        && faces[fid2].indices[k2] == e.indices.x)
                {
                    faces[fid2].edges[k1] = i;
                }
            }
        }

        let deformations = DeformationInfos {
            margin: na::convert(0.1), // FIXME: find a better way to define the margin.
            curr_timestamp: 0,
            timestamps: Vec::new(),
            ref_vertices: Vec::new(),
            tri_to_update: Vec::new(),
        };

        TriMesh {
            bvt,
            points,
            uvs,
            deformations,
            vertices,
            edges,
            faces,
            adj_face_list,
            adj_vertex_list,
            oriented: false,
        }
    }

    fn edges_list(indices: &[Point3<usize>]) -> Vec<TriMeshEdge> {
        let mut edges = HashMap::with_hasher(DeterministicState::new());

        fn key(a: usize, b: usize) -> (usize, usize) {
            if a < b {
                (a, b)
            } else {
                (b, a)
            }
        }

        for (fid, idx) in indices.iter().enumerate() {
            let e1 = key(idx.x, idx.y);
            let e2 = key(idx.y, idx.z);
            let e3 = key(idx.z, idx.x);

            match edges.entry(e1) {
                Entry::Vacant(e) => {
                    let _ = e.insert(TriMeshEdge {
                        indices: Point2::new(e1.0, e1.1),
                        adj_faces: (
                            FaceAdjacentToEdge::new(fid, 0),
                            FaceAdjacentToEdge::new(fid, 0),
                        ),
                    });
                }
                Entry::Occupied(mut e) => e.get_mut().adj_faces.1 = FaceAdjacentToEdge::new(fid, 0),
            }

            match edges.entry(e2) {
                Entry::Vacant(e) => {
                    let _ = e.insert(TriMeshEdge {
                        indices: Point2::new(e2.0, e2.1),
                        adj_faces: (
                            FaceAdjacentToEdge::new(fid, 1),
                            FaceAdjacentToEdge::new(fid, 1),
                        ),
                    });
                }
                Entry::Occupied(e) => e.into_mut().adj_faces.1 = FaceAdjacentToEdge::new(fid, 1),
            }

            match edges.entry(e3) {
                Entry::Vacant(e) => {
                    let _ = e.insert(TriMeshEdge {
                        indices: Point2::new(e3.0, e3.1),
                        adj_faces: (
                            FaceAdjacentToEdge::new(fid, 2),
                            FaceAdjacentToEdge::new(fid, 2),
                        ),
                    });
                }
                Entry::Occupied(e) => e.into_mut().adj_faces.1 = FaceAdjacentToEdge::new(fid, 2),
            }
        }

        edges.values().cloned().collect()
    }

    fn adj_vertex_list(edges: &[TriMeshEdge], vertices: &mut [TriMeshVertex]) -> Vec<usize> {
        let mut num_neighbors: Vec<usize> = iter::repeat(0).take(vertices.len()).collect();

        for e in edges {
            num_neighbors[e.indices.x] += 1;
            num_neighbors[e.indices.y] += 1;
        }

        let mut total_num_nbh = 0;

        for (num_nbh, vtx) in num_neighbors.iter().zip(vertices.iter_mut()) {
            vtx.adj_vertices = total_num_nbh..total_num_nbh + num_nbh;
            total_num_nbh += num_nbh;
        }

        let mut adj_vertex_list: Vec<usize> = iter::repeat(0).take(total_num_nbh).collect();

        // Build the adjacency list.
        for n in &mut num_neighbors {
            *n = 0;
        }

        for e in edges.iter() {
            adj_vertex_list
                [vertices[e.indices.x].adj_vertices.start + num_neighbors[e.indices.x]] =
                e.indices.y;
            adj_vertex_list
                [vertices[e.indices.y].adj_vertices.start + num_neighbors[e.indices.y]] =
                e.indices.x;

            num_neighbors[e.indices.x] += 1;
            num_neighbors[e.indices.y] += 1;
        }

        adj_vertex_list
    }

    fn adj_face_list(indices: &[Point3<usize>], vertices: &mut [TriMeshVertex]) -> Vec<usize> {
        let mut num_neighbors: Vec<usize> = iter::repeat(0).take(vertices.len()).collect();

        for idx in indices {
            num_neighbors[idx.x] += 1;
            num_neighbors[idx.y] += 1;
            num_neighbors[idx.z] += 1;
        }

        let mut total_num_nbh = 0;

        for (num_nbh, vtx) in num_neighbors.iter().zip(vertices.iter_mut()) {
            vtx.adj_faces = total_num_nbh..total_num_nbh + num_nbh;
            total_num_nbh += num_nbh;
        }

        let mut adj_face_list: Vec<usize> = iter::repeat(0).take(total_num_nbh).collect();

        // Build the adjacency list.
        for n in &mut num_neighbors {
            *n = 0;
        }

        for (i, idx) in indices.iter().enumerate() {
            adj_face_list[vertices[idx.x].adj_faces.start + num_neighbors[idx.x]] = i;
            adj_face_list[vertices[idx.y].adj_faces.start + num_neighbors[idx.y]] = i;
            adj_face_list[vertices[idx.z].adj_faces.start + num_neighbors[idx.z]] = i;

            num_neighbors[idx.x] += 1;
            num_neighbors[idx.y] += 1;
            num_neighbors[idx.z] += 1;
        }

        adj_face_list
    }

    /// The triangle mesh's AABB.
    #[inline]
    pub fn aabb(&self) -> &AABB<N> {
        self.bvt
            .root_bounding_volume()
            .expect("An empty TriMesh has no AABB.")
    }

    /// The points of this mesh.
    #[inline]
    pub fn points(&self) -> &[Point<N>] {
        &self.points
    }

    /// The faces of this mesh.
    #[inline]
    pub fn faces(&self) -> &[TriMeshFace<N>] {
        &self.faces
    }

    /// The edges of this mesh.
    #[inline]
    pub fn edges(&self) -> &[TriMeshEdge] {
        &self.edges
    }

    /// The vertices of this mesh.
    #[inline]
    pub fn vertices(&self) -> &[TriMeshVertex] {
        &self.vertices
    }

    /// Applies in-place a transformation to this triangle mesh.
    pub fn transform_by(&mut self, transform: &Isometry<N>) {
        for pt in &mut self.points {
            *pt = transform * *pt
        }
    }

    /// Applies a transformation to this triangle mesh.
    pub fn transformed(mut self, transform: &Isometry<N>) -> Self {
        self.transform_by(transform);
        self
    }

    /// Applies in-place a non-uniform scale to this triangle mesh.
    pub fn scale_by(&mut self, scale: &Vector<N>) {
        for pt in &mut self.points {
            pt.coords.component_mul_assign(scale)
        }
    }

    /// Applies a non-uniform scale to this triangle mesh.
    pub fn scaled(mut self, scale: &Vector<N>) -> Self {
        self.scale_by(scale);
        self
    }

    /// Whether this trimesh is considered is oriented or not.
    ///
    /// By default a trimesh is not oriented.
    #[inline]
    pub fn oriented(&self) -> bool {
        self.oriented
    }

    /// Whether this trimesh is considered as oriented or not.
    ///
    /// This is determined at the initialization of the trimesh.
    #[inline]
    pub fn set_oriented(&mut self, oriented: bool) {
        self.oriented = oriented
    }

    /// Face containing feature.
    #[inline]
    pub fn face_containing_feature(&self, id: FeatureId) -> usize {
        match id {
            FeatureId::Vertex(i) => self.adj_face_list[self.vertices[i].adj_faces.start],
            FeatureId::Edge(i) => self.edges[i].adj_faces.0.face_id,
            FeatureId::Face(i) => i % self.faces.len(),
            _ => panic!("Feature ID cannot be unknown."),
        }
    }

    /// The segment of the `i`-th edge on this triangle mesh.
    #[inline]
    pub fn edge_segment(&self, i: usize) -> Segment<N> {
        let edge = &self.edges[i];
        Segment::new(self.points[edge.indices.x], self.points[edge.indices.y])
    }

    /// The texture coordinates of this mesh.
    #[inline]
    pub fn uvs(&self) -> Option<&[Point2<N>]> {
        self.uvs.as_ref().map(|uvs| &uvs[..])
    }

    /// Gets the i-th mesh element.
    #[inline]
    pub fn triangle_at(&self, i: usize) -> Triangle<N> {
        let idx = self.faces[i].indices;

        Triangle::new(self.points[idx.x], self.points[idx.y], self.points[idx.z])
    }

    /// Returns `true` if the given feature is a FeatureId::Face and
    /// identifies a backface of this trimesh.
    #[inline]
    pub fn is_backface(&self, feature: FeatureId) -> bool {
        if let FeatureId::Face(i) = feature {
            i >= self.faces.len()
        } else {
            false
        }
    }

    /// The optimization structure used by this triangle mesh.
    #[inline]
    pub fn bvt(&self) -> &BVT<usize, AABB<N>> {
        &self.bvt
    }

    /// Tests that the given `dir` is on the tangent cone of the `i`th vertex
    /// of this mesh.
    pub fn vertex_tangent_cone_contains_dir(
        &self,
        i: usize,
        deformations: Option<&[N]>,
        dir: &Unit<Vector<N>>,
    ) -> bool {
        if !self.oriented {
            return false;
        }

        let v = &self.vertices[i];

        if let Some(coords) = deformations {
            for adj_face in &self.adj_face_list[v.adj_faces.clone()] {
                let indices = self.faces[*adj_face].indices * DIM;
                let tri = Triangle::new(
                    Point::from_slice(&coords[indices.x..indices.x + DIM]),
                    Point::from_slice(&coords[indices.y..indices.y + DIM]),
                    Point::from_slice(&coords[indices.z..indices.z + DIM]),
                );

                if tri.scaled_normal().dot(dir) > N::zero() {
                    return false;
                }
            }
        } else {
            for adj_face in &self.adj_face_list[v.adj_faces.clone()] {
                let face = &self.faces[*adj_face];

                if let Some(ref n) = face.normal {
                    if n.dot(dir) > N::zero() {
                        return false;
                    }
                }
            }
        }

        true
    }

    /// Tests that the given `dir` is on the polar of the tangent cone of the `i`th vertex
    /// of this mesh.
    pub fn vertex_tangent_cone_polar_contains_dir(
        &self,
        i: usize,
        dir: &Unit<Vector<N>>,
        sin_ang_tol: N,
    ) -> bool {
        let v = &self.vertices[i];

        for adj_vtx in &self.adj_vertex_list[v.adj_vertices.clone()] {
            let edge_dir = self.points[i] - self.points[*adj_vtx];

            // FIXME: don't compute the norm every time.
            if edge_dir.dot(dir) < -sin_ang_tol * edge_dir.norm() {
                return false;
            }
        }

        true
    }

    /// Tests that the given `dir` is on the tangent cone of the `i`th edge
    /// of this mesh.
    pub fn edge_tangent_cone_contains_dir(
        &self,
        i: usize,
        deformations: Option<&[N]>,
        dir: &Unit<Vector<N>>,
    ) -> bool {
        if !self.oriented {
            return false;
        }

        let e = &self.edges[i];

        if let Some(coords) = deformations {
            for adj_face in [e.adj_faces.0.face_id, e.adj_faces.1.face_id].iter() {
                let indices = self.faces[*adj_face].indices * DIM;
                let tri = Triangle::new(
                    Point::from_slice(&coords[indices.x..indices.x + DIM]),
                    Point::from_slice(&coords[indices.y..indices.y + DIM]),
                    Point::from_slice(&coords[indices.z..indices.z + DIM]),
                );

                if tri.scaled_normal().dot(dir) > N::zero() {
                    return false;
                }
            }
        } else {
            for adj_face in [e.adj_faces.0.face_id, e.adj_faces.1.face_id].iter() {
                let face = &self.faces[*adj_face];

                if let Some(ref n) = face.normal {
                    if n.dot(dir) > N::zero() {
                        return false;
                    }
                }
            }
        }

        true
    }

    /// Tests that the given `dir` is on the polar of the tangent cone of the `i`th edge
    /// of this mesh.
    ///
    /// The `dir` is assumed to be orthogonal to the edge.
    pub fn edge_tangent_cone_polar_contains_orthogonal_dir(
        &self,
        i: usize,
        dir: &Unit<Vector<N>>,
        sin_ang_tol: N,
    ) -> bool {
        let e = &self.edges[i];
        let f1 = &self.faces[e.adj_faces.0.face_id];
        let f2 = &self.faces[e.adj_faces.1.face_id];

        if let Some(side_normal1) = f1.side_normals.as_ref() {
            if side_normal1[e.adj_faces.0.edge_id].dot(dir) <= na::convert(-sin_ang_tol) {
                return false;
            }
        }

        if let Some(side_normal2) = f2.side_normals.as_ref() {
            if side_normal2[e.adj_faces.1.edge_id].dot(dir) <= na::convert(-sin_ang_tol) {
                return false;
            }
        }

        if let (Some(n1), Some(n2)) = (f1.normal, f2.normal) {
            if (n1.into_inner() + n2.into_inner()).dot(dir) < N::zero() {
                return false;
            }
        }

        true
    }

    /// Tests that the given `dir` is on the polar of the tangent cone of the `i`th edge
    /// of this mesh.
    pub fn edge_tangent_cone_polar_contains_dir(
        &self,
        i: usize,
        dir: &Unit<Vector<N>>,
        sin_ang_tol: N,
        _cos_ang_tol: N,
    ) -> bool {
        let e = &self.edges[i];
        let edge_dir = self.points[e.indices.y] - self.points[e.indices.x];

        edge_dir.dot(dir).abs() <= sin_ang_tol * edge_dir.norm()
            && self.edge_tangent_cone_polar_contains_orthogonal_dir(i, dir, sin_ang_tol)
    }

    /// Tests that the given `dir` is on the tangent cone of the `i`th face
    /// of this mesh.
    pub fn face_tangent_cone_contains_dir(
        &self,
        i: usize,
        deformations: Option<&[N]>,
        dir: &Unit<Vector<N>>,
    ) -> bool {
        if !self.oriented {
            return false;
        }

        let normal;

        if let Some(coords) = deformations {
            let indices = self.faces[i % self.faces.len()].indices * DIM;
            let tri = Triangle::new(
                Point::from_slice(&coords[indices.x..indices.x + DIM]),
                Point::from_slice(&coords[indices.y..indices.y + DIM]),
                Point::from_slice(&coords[indices.z..indices.z + DIM]),
            );

            if i >= self.faces.len() {
                normal = -tri.scaled_normal();
            } else {
                normal = tri.scaled_normal();
            }
        } else {
            if i >= self.faces.len() {
                normal = -self.faces[i - self.faces.len()]
                    .normal
                    .map(|n| n.into_inner())
                    .unwrap_or(Vector::zeros());
            } else {
                normal = self.faces[i]
                    .normal
                    .map(|n| n.into_inner())
                    .unwrap_or(Vector::zeros());
            }
        }

        normal.dot(dir) <= N::zero()
    }

    /// Checks if the polar of the tangent cone of the `i`-th face of this triangle mesh contains
    /// the specified direction within an angular tolerence.
    pub fn face_tangent_cone_polar_contains_dir(
        &self,
        i: usize,
        dir: &Unit<Vector<N>>,
        cos_ang_tol: N,
    ) -> bool {
        let normal;

        if i >= self.faces.len() {
            normal = -self.faces[i - self.faces.len()]
                .normal
                .map(|n| n.into_inner())
                .unwrap_or(Vector::zeros());
        } else {
            normal = self.faces[i]
                .normal
                .map(|n| n.into_inner())
                .unwrap_or(Vector::zeros());
        }

        normal.dot(dir) >= cos_ang_tol
    }

    /// Checks if the polar of the tangent cone of the specified feature of this triangle mesh contains
    /// the specified direction within an angular tolerence.
    pub fn tangent_cone_polar_contains_dir(
        &self,
        feature: FeatureId,
        dir: &Unit<Vector<N>>,
        sin_ang_tol: N,
        cos_ang_tol: N,
    ) -> bool {
        match feature {
            FeatureId::Face(i) => self.face_tangent_cone_polar_contains_dir(i, dir, cos_ang_tol),
            FeatureId::Edge(i) => {
                self.edge_tangent_cone_polar_contains_dir(i, dir, sin_ang_tol, cos_ang_tol)
            }
            FeatureId::Vertex(i) => {
                self.vertex_tangent_cone_polar_contains_dir(i, dir, sin_ang_tol)
            }
            FeatureId::Unknown => false,
        }
    }

    fn init_deformation_infos(&mut self) -> bool {
        if self.deformations.ref_vertices.is_empty() {
            self.deformations.timestamps = iter::repeat(0).take(self.faces.len()).collect();
            self.deformations.ref_vertices = self.points.clone();
            true
        } else {
            false
        }
    }
}

impl<N: RealField> CompositeShape<N> for TriMesh<N> {
    #[inline]
    fn nparts(&self) -> usize {
        self.faces.len()
    }

    #[inline(always)]
    fn map_part_at(
        &self,
        i: usize,
        m: &Isometry<N>,
        f: &mut dyn FnMut(&Isometry<N>, &dyn Shape<N>),
    ) {
        let element = self.triangle_at(i);
        f(m, &element)
    }

    fn map_part_and_preprocessor_at(
        &self,
        i: usize,
        m: &Isometry<N>,
        prediction: &ContactPrediction<N>,
        f: &mut dyn FnMut(&Isometry<N>, &dyn Shape<N>, &dyn ContactPreprocessor<N>),
    ) {
        let element = self.triangle_at(i);
        let preprocessor = TriMeshContactProcessor::new(self, m, i, prediction);
        f(m, &element, &preprocessor)
    }

    #[inline]
    fn aabb_at(&self, i: usize) -> AABB<N> {
        self.bvt
            .leaf(self.faces[i].bvt_leaf)
            .bounding_volume()
            .clone()
    }

    #[inline]
    fn bvh(&self) -> BVHImpl<N, usize, AABB<N>> {
        BVHImpl::BVT(&self.bvt)
    }
}

impl<N: RealField> DeformableShape<N> for TriMesh<N> {
    fn deformations_type(&self) -> DeformationsType {
        DeformationsType::Vectors
    }

    /// Updates all the degrees of freedom of this shape.
    fn set_deformations(&mut self, coords: &[N]) {
        assert!(
            coords.len() >= self.points.len() * DIM,
            "Set deformations error: dimension mismatch."
        );

        let is_first_init = self.init_deformation_infos();
        self.deformations.curr_timestamp += 1;

        // There is a bit of unsafe code in order to perform a memcopy for
        // efficiency reasons when the mapping between degrees of freedom
        // is trivial.
        unsafe {
            let len = self.points.len();
            let coords_ptr = coords.as_ptr() as *const Point<N>;
            let coords_pt: &[Point<N>] = slice::from_raw_parts(coords_ptr, len);
            self.points.copy_from_slice(coords_pt);
        }

        for (target, pt) in self.points.iter_mut().enumerate() {
            let ref_pt = &mut self.deformations.ref_vertices[target];
            let sq_dist_to_ref = na::distance_squared(pt, ref_pt);

            if is_first_init || sq_dist_to_ref > self.deformations.margin * self.deformations.margin
            {
                // We have to update the adjacent bounding volumes.
                // Note that they can be duplicates on `tri_to_update`.
                // Those duplicates will be filtered using timestamps in the next for loop.
                let ids = self.vertices[target].adj_faces.clone();
                self.deformations
                    .tri_to_update
                    .extend_from_slice(&self.adj_face_list[ids]);
                *ref_pt = *pt;
            }
        }

        // Update normals.
        for f in &mut self.faces {
            let ab = self.points[f.indices.y] - self.points[f.indices.x];
            let ac = self.points[f.indices.z] - self.points[f.indices.x];

            if let Some(n) = Unit::try_new(ab.cross(&ac), N::default_epsilon()) {
                let bc = self.points[f.indices.z] - self.points[f.indices.y];
                f.normal = Some(n);
                f.side_normals = Some([
                    Unit::new_normalize(ab.cross(&n)),
                    Unit::new_normalize(bc.cross(&n)),
                    Unit::new_normalize(-ac.cross(&n)),
                ]);
            } else {
                f.normal = None;
                f.side_normals = None;
            }
        }

        // Apply the bounding volumes changes.
        for tri_id in self.deformations.tri_to_update.drain(..) {
            if self.deformations.timestamps[tri_id] != self.deformations.curr_timestamp {
                // Update the BV.
                let idx = &self.faces[tri_id].indices;
                let mut new_bv = bounding_volume::point_cloud_aabb(
                    &Id::new(),
                    &[self.points[idx.x], self.points[idx.y], self.points[idx.z]],
                );
                new_bv.loosen(self.deformations.margin);
                self.bvt
                    .set_leaf_bounding_volume(self.faces[tri_id].bvt_leaf, new_bv, false);
                self.deformations.timestamps[tri_id] = self.deformations.curr_timestamp;
            }
        }

        // FIXME: measure efficiency with a non-zero margin.
        self.bvt.refit(N::zero())
    }

    fn update_local_approximation(&self, coords: &[N], approx: &mut LocalShapeApproximation<N>) {
        match approx.feature {
            FeatureId::Vertex(i) => {
                approx.point = Point::from_slice(&coords[i * DIM..(i + 1) * DIM]);
                approx.geometry = NeighborhoodGeometry::Point;
            }
            FeatureId::Edge(i) => {
                let edge = &self.edges[i];
                let pid1 = edge.indices.x * DIM;
                let pid2 = edge.indices.y * DIM;
                let seg = Segment::new(
                    Point::from_slice(&coords[pid1..pid1 + DIM]),
                    Point::from_slice(&coords[pid2..pid2 + DIM]),
                );
                approx.point = *seg.a();

                if let Some(dir) = seg.direction() {
                    approx.geometry = NeighborhoodGeometry::Line(dir);
                } else {
                    approx.geometry = NeighborhoodGeometry::Point;
                }
            }
            FeatureId::Face(mut i) => {
                let is_backface = i >= self.faces.len();
                if is_backface {
                    i -= self.faces.len();
                }

                let face = &self.faces[i];
                let pid1 = face.indices.x * DIM;
                let pid2 = face.indices.y * DIM;
                let pid3 = face.indices.z * DIM;
                let tri = Triangle::new(
                    Point::from_slice(&coords[pid1..pid1 + DIM]),
                    Point::from_slice(&coords[pid2..pid2 + DIM]),
                    Point::from_slice(&coords[pid3..pid3 + DIM]),
                );

                approx.point = *tri.a();

                if let Some(n) = tri.normal() {
                    if !is_backface {
                        approx.geometry = NeighborhoodGeometry::Plane(n);
                    } else {
                        approx.geometry = NeighborhoodGeometry::Plane(-n);
                    }
                } else {
                    approx.geometry = NeighborhoodGeometry::Point;
                }
            }
            _ => panic!(
                "Encountered invalid triangle feature: {:?}.",
                approx.feature
            ),
        }
    }
}

impl<N: RealField> From<procedural::TriMesh<N>> for TriMesh<N> {
    fn from(trimesh: procedural::TriMesh<N>) -> Self {
        let indices = trimesh
            .flat_indices()
            .chunks(3)
            .map(|idx| Point3::new(idx[0] as usize, idx[1] as usize, idx[2] as usize))
            .collect();
        TriMesh::new(trimesh.coords, indices, trimesh.uvs)
    }
}

struct TriMeshContactProcessor<'a, N: RealField> {
    mesh: &'a TriMesh<N>,
    pos: &'a Isometry<N>,
    face_id: usize,
    prediction: &'a ContactPrediction<N>,
}

impl<'a, N: RealField> TriMeshContactProcessor<'a, N> {
    pub fn new(
        mesh: &'a TriMesh<N>,
        pos: &'a Isometry<N>,
        face_id: usize,
        prediction: &'a ContactPrediction<N>,
    ) -> Self {
        TriMeshContactProcessor {
            mesh,
            pos,
            face_id,
            prediction,
        }
    }
}

impl<'a, N: RealField> ContactPreprocessor<N> for TriMeshContactProcessor<'a, N> {
    fn process_contact(
        &self,
        c: &mut Contact<N>,
        kinematic: &mut ContactKinematic<N>,
        is_first: bool,
    ) -> bool {
        // Fix the feature ID.
        let feature = if is_first {
            kinematic.feature1()
        } else {
            kinematic.feature2()
        };

        let face = &self.mesh.faces()[self.face_id];
        let actual_feature = match feature {
            FeatureId::Vertex(i) => FeatureId::Vertex(face.indices[i]),
            FeatureId::Edge(i) => FeatureId::Edge(face.edges[i]),
            FeatureId::Face(i) => {
                if i == 0 {
                    FeatureId::Face(self.face_id)
                } else {
                    FeatureId::Face(self.face_id + self.mesh.faces().len())
                }
            }
            FeatureId::Unknown => FeatureId::Unknown,
        };

        if is_first {
            kinematic.set_feature1(actual_feature);
        } else {
            kinematic.set_feature2(actual_feature);
        }

        // Test the validity of the LMD.
        if c.depth > N::zero() {
            true
        } else {
            let local_dir = self.pos.inverse_transform_unit_vector(&c.normal);

            if is_first {
                self.mesh.tangent_cone_polar_contains_dir(
                    actual_feature,
                    &local_dir,
                    self.prediction.sin_angular1(),
                    self.prediction.cos_angular1(),
                )
            } else {
                self.mesh.tangent_cone_polar_contains_dir(
                    actual_feature,
                    &-local_dir,
                    self.prediction.sin_angular2(),
                    self.prediction.cos_angular2(),
                )
            }
        }
    }
}
