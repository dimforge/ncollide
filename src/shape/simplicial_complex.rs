//! 2d line strip, 3d triangle mesh, and nd subsimplex mesh.

use arrayvec::ArrayVec;
use bounding_volume::{self, AABB, BoundingVolume, CircularCone, ConicalApproximation, SpatializedNormalCone};
use math::{DIM, Isometry, Point, Vector};
use na::{self, Id, Point2, Point3, Real, Unit};
use partitioning::{BVHImpl, BVT};
use procedural;
use query::{LocalShapeApproximation, NeighborhoodGeometry};
use shape::{CompositeShape, DeformableShape, DeformationsType, FeatureId, Segment, Shape, Triangle};
use std::iter;
use std::ops::Range;
use std::slice;

#[derive(Clone)]
struct RefVertex<N: Real> {
    point: Point<N>,
    adj_bvs: Range<usize>,
}

#[derive(Clone)]
struct DeformationInfos<N: Real> {
    margin: N,
    curr_timestamp: usize,
    timestamps: Vec<usize>,
    adj_list: Vec<usize>,
    ref_vertices: Vec<RefVertex<N>>,
    tri_to_update: Vec<usize>,
}

#[derive(Clone)]
struct Vertex<N: Real> {
    adj_faces: Range<usize>,
    normals: ConicalApproximation<N>,
    bvt_leaf: usize,
}

#[derive(Clone)]
struct Edge<N: Real> {
    idx: Point2<usize>,
    adj_faces: (usize, usize),
    dir: Unit<Vector<N>>,
    normals: ConicalApproximation<N>,
    bvt_leaf: usize,
}

#[derive(Clone)]
struct Face<N: Real> {
    idx: Point3<usize>,
    normal: Option<Unit<Vector<N>>>,
    bvt_leaf: usize,
}

/// A 3d triangle mesh represented as a simplicial complex.
#[derive(Clone)]
pub struct SimplicialComplex<N: Real> {
    bvt: BVT<FeatureId, SpatializedNormalCone<N>>,
    points: Vec<Point<N>>,
    faces: Vec<Face<N>>,
    edges: Vec<Edge<N>>,
    vertices: Vec<Vertex<N>>,
    // Collection of adjacent faces for vertices.
    adj_list: Vec<usize>,
    uvs: Option<Vec<Point2<N>>>,
    deformations: DeformationInfos<N>,
}

impl<N: Real> SimplicialComplex<N> {
    /// Builds a new mesh.
    pub fn new(
        vertices: Vec<Point<N>>,
        indices: Vec<Point3<usize>>,
        uvs: Option<Vec<Point2<N>>>,
    ) -> SimplicialComplex<N> {
        /*
        let mut leaves = Vec::new();

        {
            let is = &*indices;

            for (i, is) in is.iter().enumerate() {
                let triangle = Triangle::new(vertices[is.x], vertices[is.y], vertices[is.z]);
                // FIXME: loosen for better persistency?
                let bv = triangle.aabb(&Isometry::identity());
                leaves.push((i, bv.clone()));
            }
        }

        let bvt = BVT::new_balanced(leaves);
        let mut bvt_leaf_ids: Vec<usize> = iter::repeat(0).take(bvt.leaves().len()).collect();

        for (i, leaf) in bvt.leaves().iter().enumerate() {
            bvt_leaf_ids[*leaf.data()] = i
        }

        let deformations = DeformationInfos {
            margin: na::convert(0.1), // FIXME: find a better way to defined the margin.
            curr_timestamp: 0,
            timestamps: Vec::new(),
            adj_list: Vec::new(),
            ref_vertices: Vec::new(),
            tri_to_update: Vec::new(),
        };

        SimplicialComplex {
            bvt,
            bvt_leaf_ids,
            vertices,
            indices,
            uvs,
            deformations,
        }
        */
        unimplemented!()
    }

    /// The simplicial complex's AABB.
    #[inline]
    pub fn aabb(&self) -> &AABB<N> {
        &self.bvt.root_bounding_volume().expect("An empty SimplicialComplex has no AABB.").aabb
    }

    /// The vertices of this complex.
    #[inline]
    pub fn points(&self) -> &Vec<Point<N>> {
        &self.points
    }

    /// The texture coordinates of this mesh.
    #[inline]
    pub fn uvs(&self) -> &Option<Vec<Point2<N>>> {
        &self.uvs
    }

    /// Gets the i-th mesh element.
    #[inline]
    pub fn triangle_at(&self, i: usize) -> Triangle<N> {
        let idx = self.faces[i].idx;

        Triangle::new(
            self.points[idx.x],
            self.points[idx.y],
            self.points[idx.z],
        )
    }

    /// Gets the i-th segment of this mesh.
    #[inline]
    pub fn segment_at(&self, i: usize) -> Segment<N> {
        let idx = self.edges[i].idx;

        Segment::new(
            self.points[idx.x],
            self.points[idx.y],
        )
    }

    /// Sets to `out` the local linear approximation of the feature `id` of this shape.
    ///
    /// Note that because this shape is polyhedral, this local approximation is actually exact.
    /// Returns `false` if the requested feature is a degenerate face or a degenerate edge.
    #[inline]
    pub fn local_approximation(&self, id: FeatureId, out: &mut LocalShapeApproximation<N>) -> bool {
        out.feature = id;

        match id {
            FeatureId::Face(i) => {
                let face = &self.faces[i];
                out.point = self.points[face.idx.x];

                if let Some(normal) = face.normal {
                    out.geometry = NeighborhoodGeometry::Plane(normal);
                    true
                } else {
                    false
                }
            }
            FeatureId::Edge(i) => {
                let edge = &self.edges[i];
                out.point = self.points[edge.idx.x];
                out.geometry = NeighborhoodGeometry::Line(edge.dir);
                true
            }
            FeatureId::Vertex(i) => {
                out.point = self.points[i];
                out.geometry = NeighborhoodGeometry::Point;
                true
            }
            FeatureId::Unknown => false
        }
    }

    /// The optimization structure used by this triangle mesh.
    #[inline]
    pub fn bvt(&self) -> &BVT<FeatureId, SpatializedNormalCone<N>> {
        &self.bvt
    }

    // Update normal cones after vertices have been modified.
    fn update_normals(&mut self) {
        // First, update faces normals.
        for face in &mut self.faces {
            let triangle = Triangle::new(
                self.points[face.idx.x],
                self.points[face.idx.y],
                self.points[face.idx.z],
            );

            face.normal = triangle.normal();
        }

        // Second, update edges normal cones.
        // Third, update vertices normal cones.
        for vtx in &mut self.vertices {
            vtx.normals = ConicalApproximation::Empty;

            for adj in vtx.adj_faces.clone() {
                let face = &self.faces[adj];
                if let Some(normal) = face.normal {
                    vtx.normals.push(normal)
                }
            }
        }
    }

    fn init_deformation_infos(&mut self) -> bool {
        unimplemented!()
        /*
        if self.deformations.ref_vertices.is_empty() {
            self.deformations.ref_vertices = Vec::with_capacity(self.vertices.len());
            self.deformations.timestamps = iter::repeat(0).take(self.indices.len()).collect();
            let mut num_neighbors: Vec<usize> = iter::repeat(0).take(self.vertices.len()).collect();

            for idx in &self.indices {
                num_neighbors[idx.x] += 1;
                num_neighbors[idx.y] += 1;
                num_neighbors[idx.z] += 1;
            }

            let mut total_num_nbh = 0;

            for (num_nbh, pt) in num_neighbors.iter().zip(self.vertices.iter()) {
                self.deformations.ref_vertices.push(RefVertex {
                    point: *pt,
                    adj_bvs: total_num_nbh..total_num_nbh + num_nbh,
                });
                total_num_nbh += num_nbh;
            }

            self.deformations.adj_list = iter::repeat(0).take(total_num_nbh).collect();

            // Build the adjascency list.
            for n in &mut num_neighbors {
                *n = 0;
            }

            for (i, idx) in self.indices.iter().enumerate() {
                self.deformations.adj_list[self.deformations.ref_vertices[idx.x].adj_bvs.start + num_neighbors[idx.x]] = i;
                self.deformations.adj_list[self.deformations.ref_vertices[idx.y].adj_bvs.start + num_neighbors[idx.y]] = i;
                self.deformations.adj_list[self.deformations.ref_vertices[idx.z].adj_bvs.start + num_neighbors[idx.z]] = i;

                num_neighbors[idx.x] += 1;
                num_neighbors[idx.y] += 1;
                num_neighbors[idx.z] += 1;
            }

            true
        } else {
            false
        }
        */
    }
}

/*
impl<N: Real> CompositeShape<N> for SimplicialComplex<N> {
    #[inline]
    fn nparts(&self) -> usize {
        self.faces.len()
    }

    #[inline(always)]
    fn map_part_at(&self, i: usize, f: &mut FnMut(usize, &Isometry<N>, &Shape<N>)) {
        self.map_transformed_part_at(i, &Isometry::identity(), f)
    }

    #[inline(always)]
    fn map_transformed_part_at(
        &self,
        i: usize,
        m: &Isometry<N>,
        f: &mut FnMut(usize, &Isometry<N>, &Shape<N>),
    ) {
        let element = self.triangle_at(i);

        f(i, m, &element)
    }

    #[inline]
    fn aabb_at(&self, i: usize) -> AABB<N> {
        self.bvt.leaf(self.bvt_leaf_ids[i]).bounding_volume().clone()
    }

    #[inline]
    fn bvh(&self) -> BVHImpl<N, usize, AABB<N>> {
        BVHImpl::BVT(&self.bvt)
    }
}

impl<N: Real> DeformableShape<N> for SimplicialComplex<N> {
    fn deformations_type(&self) -> DeformationsType {
        DeformationsType::Vectors
    }

    /// Updates all the degrees of freedom of this shape.
    fn set_deformations(&mut self, coords: &[N], indices: Option<&[usize]>) {
        let is_first_init = self.init_deformation_infos();
        self.deformations.curr_timestamp += 1;

        if indices.is_none() {
            // There is a bit of unsafe code in order to perform a memcopy for
            // efficiency reasons when the mapping between degrees of freedom
            // is trivial.
            unsafe {
                let len = coords.len() / DIM;
                let coords_ptr = coords.as_ptr() as *const Point<N>;
                let coords_pt: &[Point<N>] = slice::from_raw_parts(coords_ptr, len);
                self.vertices.copy_from_slice(coords_pt);
            }
        }

        for (target, pt) in self.vertices.iter_mut().enumerate() {
            if let Some(idx) = indices {
                let source = idx[target];
                pt.coords.copy_from_slice(&coords[source..source + DIM]);
            }

            let ref_pt = &mut self.deformations.ref_vertices[target];
            let sq_dist_to_ref = na::distance_squared(pt, &ref_pt.point);

            if is_first_init || sq_dist_to_ref > self.deformations.margin * self.deformations.margin {
                // We have to update the adjacent bounding volumes.
                self.deformations.tri_to_update.extend_from_slice(&self.deformations.adj_list[ref_pt.adj_bvs.clone()]);
                ref_pt.point = *pt;
            }
        }

        // Apply the bounding volumes changes.
        for tri_id in self.deformations.tri_to_update.drain(..) {
            if self.deformations.timestamps[tri_id] != self.deformations.curr_timestamp {
                // Update the BV.
                let idx = &self.indices[tri_id];
                let mut new_bv = bounding_volume::point_cloud_aabb(&Id::new(), &[self.vertices[idx.x], self.vertices[idx.y], self.vertices[idx.z]]);
                new_bv.loosen(self.deformations.margin);
                self.bvt.set_leaf_bounding_volume(self.bvt_leaf_ids[tri_id], new_bv, false);
                self.deformations.timestamps[tri_id] = self.deformations.curr_timestamp;
            }
        }

        // FIXME: measure efficiency with a non-zero margin.
        self.bvt.refit(N::zero())
    }

    fn update_local_approximation(
        &self,
        coords: &[N],
        indices: Option<&[usize]>,
        part_id: usize,
        approx: &mut LocalShapeApproximation<N>,
    ) {
        let tri_id = self.indices[part_id];
        let (a_id, b_id, c_id) = if let Some(idx) = indices {
            (idx[tri_id.x], idx[tri_id.y], idx[tri_id.z])
        } else {
            (tri_id.x * 3, tri_id.y * 3, tri_id.z * 3)
        };

        let a = Point3::new(coords[a_id + 0], coords[a_id + 1], coords[a_id + 2]);
        let b = Point3::new(coords[b_id + 0], coords[b_id + 1], coords[b_id + 2]);
        let c = Point3::new(coords[c_id + 0], coords[c_id + 1], coords[c_id + 2]);
        let tri = Triangle::new(a, b, c);

        tri.update_local_approximation(approx);
    }
}*/

impl<N: Real> From<procedural::TriMesh<N>> for SimplicialComplex<N> {
    fn from(trimesh: procedural::TriMesh<N>) -> Self {
        let indices =
            trimesh
                .flat_indices()
                .chunks(3).map(|idx| Point3::new(idx[0] as usize, idx[1] as usize, idx[2] as usize))
                .collect();
        SimplicialComplex::new(trimesh.coords, indices, trimesh.uvs)
    }
}