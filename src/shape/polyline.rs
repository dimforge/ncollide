//! 2d line strip, 3d polyline.

use bounding_volume::{self, BoundingVolume, AABB};
use math::{Isometry, Point, Vector, DIM};
use na::{self, Id, Point2, Point3, Real, Unit};
use partitioning::{BVHImpl, BVT};
use procedural;
use query::{LocalShapeApproximation, NeighborhoodGeometry, ContactPreprocessor, ContactPrediction};
use shape::{
    CompositeShape, DeformableShape, DeformationsType, FeatureId, Segment, Shape, Triangle,
};
use std::collections::{hash_map::Entry, HashMap};
use std::iter;
use std::ops::Range;
use std::slice;
use utils::DeterministicState;

#[derive(Clone)]
struct DeformationInfos<N: Real> {
    margin: N,
    curr_timestamp: usize,
    timestamps: Vec<usize>,
    ref_vertices: Vec<Point<N>>,
    seg_to_update: Vec<usize>,
}

#[derive(Clone)]
pub struct PolylineEdge<N: Real> {
    pub indices: Point2<usize>,
    bvt_leaf: usize,
    pub normal: Option<Unit<Vector<N>>>, // FIXME: useless in 3D
}

#[derive(Clone)]
pub struct PolylineVertex {
    pub adj_edges: Range<usize>,
    pub adj_vertices: Range<usize>,
}

/// A polygonal line.
#[derive(Clone)]
pub struct Polyline<N: Real> {
    bvt: BVT<usize, AABB<N>>,
    points: Vec<Point<N>>,
    vertices: Vec<PolylineVertex>,
    edges: Vec<PolylineEdge<N>>,
    adj_edge_list: Vec<usize>,
    // NOTE: the adj_vertex_list could be deduced from the adj_edge_list.
    adj_vertex_list: Vec<usize>,
    deformations: DeformationInfos<N>,
    oriented: bool, // FIXME: useless in 3D
}

impl<N: Real> Polyline<N> {
    /// Builds a new polyline.
    pub fn new(
        points: Vec<Point<N>>,
        indices: Option<Vec<Point2<usize>>>,
    ) -> Polyline<N>
    {
        let indices = indices.unwrap_or((0..).map(|i| Point2::new(i, i + 1)).take(points.len() - 1).collect());
        let mut leaves = Vec::with_capacity(indices.len());
        let mut vertices: Vec<PolylineVertex> = iter::repeat(PolylineVertex {
            adj_edges: 0..0,
            adj_vertices: 0..0,
        })
            .take(points.len())
            .collect();
        let mut edges = Vec::with_capacity(indices.len());

        let adj_edge_list = Self::adj_edge_list(&indices, &mut vertices);
        let adj_vertex_list = Self::adj_vertex_list(&indices, &mut vertices);

        {
            let is = &*indices;

            for (i, is) in is.iter().enumerate() {
                let segment = Segment::new(points[is.x], points[is.y]);
                let normal = segment.normal();

                let bv = segment.aabb(&Isometry::identity());
                leaves.push((i, bv.clone()));
                edges.push(PolylineEdge {
                    indices: *is,
                    bvt_leaf: 0,             // Will be set later.
                    normal,
                })
            }
        }

        let bvt = BVT::new_balanced(leaves);

        // Set edge.bvt_leaf
        for (i, leaf) in bvt.leaves().iter().enumerate() {
            edges[*leaf.data()].bvt_leaf = i;
        }

        let deformations = DeformationInfos {
            margin: na::convert(0.1), // FIXME: find a better way to define the margin.
            curr_timestamp: 0,
            timestamps: Vec::new(),
            ref_vertices: Vec::new(),
            seg_to_update: Vec::new(),
        };

        Polyline {
            bvt,
            points,
            deformations,
            vertices,
            edges,
            adj_edge_list,
            adj_vertex_list,
            oriented: false,
        }
    }

    fn adj_vertex_list(edges: &[Point2<usize>], vertices: &mut [PolylineVertex]) -> Vec<usize> {
        let mut num_neighbors: Vec<usize> = iter::repeat(0).take(vertices.len()).collect();

        for e in edges {
            num_neighbors[e.x] += 1;
            num_neighbors[e.y] += 1;
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
                [vertices[e.x].adj_vertices.start + num_neighbors[e.x]] = e.y;
            adj_vertex_list
                [vertices[e.y].adj_vertices.start + num_neighbors[e.y]] = e.x;

            num_neighbors[e.x] += 1;
            num_neighbors[e.y] += 1;
        }

        adj_vertex_list
    }

    fn adj_edge_list(edges: &[Point2<usize>], vertices: &mut [PolylineVertex]) -> Vec<usize> {
        let mut num_neighbors: Vec<usize> = iter::repeat(0).take(vertices.len()).collect();

        for idx in edges {
            num_neighbors[idx.x] += 1;
            num_neighbors[idx.y] += 1;
        }

        let mut total_num_nbh = 0;

        for (num_nbh, vtx) in num_neighbors.iter().zip(vertices.iter_mut()) {
            vtx.adj_edges = total_num_nbh..total_num_nbh + num_nbh;
            total_num_nbh += num_nbh;
        }

        let mut adj_edge_list: Vec<usize> = iter::repeat(0).take(total_num_nbh).collect();

        // Build the adjacency list.
        for n in &mut num_neighbors {
            *n = 0;
        }

        for (i, idx) in edges.iter().enumerate() {
            adj_edge_list[vertices[idx.x].adj_edges.start + num_neighbors[idx.x]] = i;
            adj_edge_list[vertices[idx.y].adj_edges.start + num_neighbors[idx.y]] = i;

            num_neighbors[idx.x] += 1;
            num_neighbors[idx.y] += 1;
        }

        adj_edge_list
    }

    /// The polyline's AABB.
    #[inline]
    pub fn aabb(&self) -> &AABB<N> {
        self.bvt
            .root_bounding_volume()
            .expect("An empty Polyline has no AABB.")
    }

    /// The points of this polyline.
    #[inline]
    pub fn points(&self) -> &[Point<N>] {
        &self.points
    }

    /// The edges of this polyline.
    #[inline]
    pub fn edges(&self) -> &[PolylineEdge<N>] {
        &self.edges
    }

    /// Whether this polyline is considered is oriented or not.
    ///
    /// By default a polyline is not oriented.
    #[inline]
    pub fn oriented(&self) -> bool {
        self.oriented
    }

    /// Whether this polyline is considered as oriented or not.
    ///
    /// This is determined at the initialization of the polyline.
    #[inline]
    pub fn set_oriented(&mut self, oriented: bool) {
        self.oriented = oriented
    }

    /// Face containing feature.
    #[inline]
    pub fn face_containing_feature(&self, id: FeatureId) -> usize {
        unimplemented!()
        /*
        match id {
            FeatureId::Vertex(i) => self.adj_edge_list[self.vertices[i].adj_edges.start],
            FeatureId::Edge(i) => self.edges[i].adj_edges.0.face_id,
            FeatureId::Face(i) => i % self.faces.len(),
            _ => panic!("Feature ID cannot be unknown."),
        }
        */
    }

    /// The segment of the `i`-th edge on this polyline.
    #[inline]
    pub fn edge_segment(&self, i: usize) -> Segment<N> {
        let edge = &self.edges[i];
        Segment::new(self.points[edge.indices.x], self.points[edge.indices.y])
    }

    /// Gets the i-th polyline element.
    #[inline]
    pub fn segment_at(&self, i: usize) -> Segment<N> {
        let idx = self.edges[i].indices;
        Segment::new(self.points[idx.x], self.points[idx.y])
    }

    /// The optimization structure used by this polyline.
    #[inline]
    pub fn bvt(&self) -> &BVT<usize, AABB<N>> {
        &self.bvt
    }

    /// Tests that the given `dir` is on the tangent cone of the `i`th vertex
    /// of this polyline.
    #[cfg(feature = "dim3")]
    pub fn vertex_tangent_cone_contains_dir(
        &self,
        i: usize,
        deformations: Option<&[N]>,
        dir: &Unit<Vector<N>>,
    ) -> bool
    {
        return false;
    }

    /// Tests that the given `dir` is on the tangent cone of the `i`th vertex
    /// of this polyline.
    #[cfg(feature = "dim2")]
    pub fn vertex_tangent_cone_contains_dir(
        &self,
        i: usize,
        deformations: Option<&[N]>,
        dir: &Unit<Vector<N>>,
    ) -> bool
    {
        if !self.oriented {
            return false;
        }

        let v = &self.vertices[i];

        if let Some(coords) = deformations {
            for adj_edge in &self.adj_edge_list[v.adj_edges.clone()] {
                let indices = self.edges[*adj_edge].indices * DIM;
                let seg = Segment::new(
                    Point::from_slice(&coords[indices.x..indices.x + DIM]),
                    Point::from_slice(&coords[indices.y..indices.y + DIM]),
                );

                if seg.scaled_normal().dot(dir) > N::zero() {
                    return false;
                }
            }
        } else {
            for adj_edge in &self.adj_edge_list[v.adj_edges.clone()] {
                let edge = &self.edges[*adj_edge];

                if let Some(ref n) = edge.normal {
                    if n.dot(dir) > N::zero() {
                        return false;
                    }
                }
            }
        }

        true
    }

    /// Tests that the given `dir` is on the polar of the tangent cone of the `i`th vertex
    /// of this polyline.
    pub fn vertex_tangent_cone_polar_contains_dir(
        &self,
        i: usize,
        dir: &Unit<Vector<N>>,
        sin_ang_tol: N,
    ) -> bool
    {
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
    /// of this polyline.
    #[cfg(feature = "dim3")]
    pub fn edge_tangent_cone_contains_dir(
        &self,
        i: usize,
        deformations: Option<&[N]>,
        dir: &Unit<Vector<N>>,
    ) -> bool
    {
        return false;
    }

    /// Tests that the given `dir` is on the tangent cone of the `i`th edge
    /// of this polyline.
    #[cfg(feature = "dim2")]
    pub fn edge_tangent_cone_contains_dir(
        &self,
        i: usize,
        deformations: Option<&[N]>,
        dir: &Unit<Vector<N>>,
    ) -> bool
    {
        if !self.oriented {
            return false;
        }

        let normal;

        if let Some(coords) = deformations {
            let indices = self.edges[i % self.edges.len()].indices * DIM;
            let seg = Segment::new(
                Point::from_slice(&coords[indices.x..indices.x + DIM]),
                Point::from_slice(&coords[indices.y..indices.y + DIM]),
            );

            if i >= self.edges.len() {
                normal = -seg.scaled_normal();
            } else {
                normal = seg.scaled_normal();
            }
        } else {
            if i >= self.edges.len() {
                normal = -self.edges[i - self.edges.len()]
                    .normal
                    .map(|n| n.unwrap())
                    .unwrap_or(Vector::zeros());
            } else {
                normal = self.edges[i]
                    .normal
                    .map(|n| n.unwrap())
                    .unwrap_or(Vector::zeros());
            }
        }

        normal.dot(dir) <= N::zero()
    }

    pub fn edge_tangent_cone_polar_contains_dir(
        &self, i: usize, dir: &Unit<Vector<N>>, cos_ang_tol: N
    ) -> bool {
        let normal;

        if i >= self.edges.len() {
            normal = -self.edges[i - self.edges.len()]
                .normal
                .map(|n| n.unwrap())
                .unwrap_or(Vector::zeros());
        } else {
            normal = self.edges[i]
                .normal
                .map(|n| n.unwrap())
                .unwrap_or(Vector::zeros());
        }

        normal.dot(dir) >= cos_ang_tol
    }

    pub fn tangent_cone_polar_contains_dir(&self, feature: FeatureId, dir: &Unit<Vector<N>>, sin_ang_tol: N, cos_ang_tol: N) -> bool {
        unimplemented!()
        /*
        match feature {
            FeatureId::Edge(i) => self.edge_tangent_cone_polar_contains_dir(i, dir, cos_ang_tol),
            FeatureId::Vertex(i) => self.vertex_tangent_cone_polar_contains_dir(i, dir, sin_ang_tol),
            FeatureId::Unknown => false
        }
        */
    }

    fn init_deformation_infos(&mut self) -> bool {
        if self.deformations.ref_vertices.is_empty() {
            self.deformations.timestamps = iter::repeat(0).take(self.edges.len()).collect();
            self.deformations.ref_vertices = self.points.clone();
            true
        } else {
            false
        }
    }
}

impl<N: Real> CompositeShape<N> for Polyline<N> {
    #[inline]
    fn nparts(&self) -> usize {
        self.edges.len()
    }

    #[inline(always)]
    fn map_part_at(
        &self,
        i: usize,
        m: &Isometry<N>,
        f: &mut FnMut(&Isometry<N>, &Shape<N>),
    )
    {
        let element = self.segment_at(i);
        f(m, &element)
    }

    fn map_part_with_preprocessor_at(
        &self,
        i: usize,
        m: &Isometry<N>,
        prediction: &ContactPrediction<N>,
        f: &mut FnMut(&Isometry<N>, &Shape<N>, &ContactPreprocessor<N>),
    ) {
        unimplemented!()
    }

    #[inline]
    fn aabb_at(&self, i: usize) -> AABB<N> {
        self.bvt
            .leaf(self.edges[i].bvt_leaf)
            .bounding_volume()
            .clone()
    }

    #[inline]
    fn bvh(&self) -> BVHImpl<N, usize, AABB<N>> {
        BVHImpl::BVT(&self.bvt)
    }
}

impl<N: Real> DeformableShape<N> for Polyline<N> {
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
                self.points.copy_from_slice(coords_pt);
            }
        }

        for (target, pt) in self.points.iter_mut().enumerate() {
            if let Some(idx) = indices {
                let source = idx[target];
                pt.coords.copy_from_slice(&coords[source..source + DIM]);
            }

            let ref_pt = &mut self.deformations.ref_vertices[target];
            let sq_dist_to_ref = na::distance_squared(pt, ref_pt);

            if is_first_init || sq_dist_to_ref > self.deformations.margin * self.deformations.margin
                {
                    // We have to update the adjacent bounding volumes.
                    // Note that they can be duplicates on `seg_to_update`.
                    // Those duplicates will be filtered using timestamps in the next for loop.
                    let ids = self.vertices[target].adj_edges.clone();
                    self.deformations
                        .seg_to_update
                        .extend_from_slice(&self.adj_edge_list[ids]);
                    *ref_pt = *pt;
                }
        }

        // Update normals.
        for e in &mut self.edges {
            let seg = Segment::new(self.points[e.indices.x], self.points[e.indices.y]);
            e.normal = seg.normal();
        }

        // Apply the bounding volumes changes.
        for seg_id in self.deformations.seg_to_update.drain(..) {
            if self.deformations.timestamps[seg_id] != self.deformations.curr_timestamp {
                // Update the BV.
                let idx = &self.edges[seg_id].indices;
                let mut new_bv = bounding_volume::point_cloud_aabb(
                    &Id::new(),
                    &[self.points[idx.x], self.points[idx.y]],
                );
                new_bv.loosen(self.deformations.margin);
                self.bvt
                    .set_leaf_bounding_volume(self.edges[seg_id].bvt_leaf, new_bv, false);
                self.deformations.timestamps[seg_id] = self.deformations.curr_timestamp;
            }
        }

        // FIXME: measure efficiency with a non-zero margin.
        self.bvt.refit(N::zero())
    }

    fn update_local_approximation(
        &self,
        coords: &[N],
        indices: Option<&[usize]>,
        approx: &mut LocalShapeApproximation<N>,
    )
    {
        unimplemented!()
        /*
        assert!(
            indices.is_none(),
            "Remapping indices are not yet supported."
        );

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
        */
    }
}

/*
impl<N: Real> From<procedural::Polyline<N>> for Polyline<N> {
    fn from(polyline: procedural::Polyline<N>) -> Self {
        let indices = polyline
            .flat_indices()
            .chunks(3)
            .map(|idx| Point3::new(idx[0] as usize, idx[1] as usize, idx[2] as usize))
            .collect();
        Polyline::new(polyline.coords, indices, polyline.uvs)
    }
}
*/
