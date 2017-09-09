use std::mem;
use std::cmp::Ordering;
use std::collections::{HashMap, HashSet, BinaryHeap};
use std::collections::hash_map::Entry;
use num::{Zero, Bounded};

use alga::general::{Id, Real};
use na::{Translation3, Point3, Vector2, Vector3};
use na;

use utils;
use geometry::shape::SupportMap;
use geometry::bounding_volume::{self, BoundingVolume, AABB};
use geometry::partitioning::{BVT, BoundingVolumeInterferencesCollector};
use geometry::query::algorithms::johnson_simplex::JohnsonSimplex;
use geometry::query::{ray_internal, Ray, RayCast, RayIntersection};
use procedural::{TriMesh, IndexBuffer};

/// Approximate convex decomposition of a triangle mesh.
pub fn hacd<N: Real>(mesh:           TriMesh<Point3<N>>,
                     error:          N,
                     min_components: usize)
                     -> (Vec<TriMesh<Point3<N>>>, Vec<Vec<usize>>) {
    assert!(mesh.normals.is_some(), "Vertex normals are required to compute the convex decomposition.");

    let mut mesh = mesh;

    let mut edges      = BinaryHeap::new();
    let (center, diag) = normalize(&mut mesh);
    let (rays, raymap) = compute_rays(&mesh);
    let mut dual_graph = compute_dual_graph(&mesh, &raymap);
    let bvt            = compute_ray_bvt(&rays[..]);

    /*
     * Initialize the binary heap.
     */
    for (i, v) in dual_graph.iter().enumerate() {
        for n in v.neighbors.as_ref().unwrap().iter() {
            if i < *n {
                let edge = DualGraphEdge::new(0, i, *n, &dual_graph[..], &mesh.coords[..], error);

                edges.push(edge);
            }
        }
    }

    /*
     * Decimation.
     */
    let mut curr_time = 0;

    while dual_graph.len() - curr_time > min_components {
        let to_add = match edges.pop() {
            None          => break,
            Some(mut top) => {
                if !top.is_valid(&dual_graph[..]) {
                    continue; // this edge has been invalidated.
                }

                if !top.exact {
                    // the cost is just an upper bound.
                    let _max: N = Bounded::max_value();
                    let mut top_cost = -_max;

                    loop {
                        let remove = match edges.peek() {
                            None        => false,
                            Some(ref e) => {
                                if !top.is_valid(&dual_graph[..]) {
                                    true
                                }
                                else {
                                    top_cost = e.mcost;
                                    false
                                }
                            }
                        };

                        if remove {
                            let _ = edges.pop();
                        }
                        else {
                            break;
                        }
                    }

                    // Compute a bounded decimation cost.
                    top.compute_decimation_cost(&dual_graph[..], &rays[..], &bvt, -top_cost, error);

                    if top.concavity > error {
                        continue;
                    }

                    if top.mcost < top_cost {
                        // we are not the greatest cost any more.
                        edges.push(top);
                        continue;
                    }
                }

                // Ensure the exact decimation cost (up to the maximum concavity) has been computed.
                top.compute_decimation_cost(&dual_graph[..], &rays[..], &bvt, Bounded::max_value(), error);

                if top.concavity > error {
                    continue;
                }

                curr_time = curr_time + 1;

                let v1 = top.v1;
                let v2 = top.v2;

                DualGraphVertex::hecol(top, &mut dual_graph[..]);

                assert!(dual_graph[v1].timestamp != Bounded::max_value());
                assert!(dual_graph[v2].timestamp != Bounded::max_value());
                dual_graph[v1].timestamp = curr_time;
                dual_graph[v2].timestamp = Bounded::max_value(); // Mark as invalid.

                v1
            }
        };

        for n in dual_graph[to_add].neighbors.as_ref().unwrap().iter() {
            let edge = DualGraphEdge::new(curr_time, to_add, *n, &dual_graph[..], &mesh.coords[..], error);

            edges.push(edge);
        }
    }

    /*
     * Build the decomposition from the remaining vertices.
     */
    let mut result = Vec::with_capacity(dual_graph.len() - curr_time);
    let mut parts  = Vec::with_capacity(dual_graph.len() - curr_time);

    for vertex in dual_graph.into_iter() {
        if vertex.timestamp != Bounded::max_value() {
            let mut chull = vertex.chull.unwrap();

            denormalize(&mut chull, &center, diag);
            result.push(chull);
            parts.push(vertex.parts.unwrap());
        }
    }

    (result, parts)
}

fn normalize<N: Real>(mesh: &mut TriMesh<Point3<N>>) -> (Point3<N>, N) {
    let (mins, maxs) = bounding_volume::point_cloud_aabb(&Id::new(), &mesh.coords[..]);
    let diag   = na::distance(&mins, &maxs);
    let center = na::center(&mins, &maxs);

    mesh.translate_by(&Translation3::from_vector(-center.coords));
    let _1: N = na::one();
    mesh.scale_by_scalar(_1 / diag);

    (center, diag)
}

fn denormalize<N: Real>(mesh: &mut TriMesh<Point3<N>>, center: &Point3<N>, diag: N) {
    mesh.scale_by_scalar(diag);
    mesh.translate_by(&Translation3::from_vector(center.coords));
}

struct DualGraphVertex<N: Real> {
    // XXX: Loss of determinism because of the randomized HashSet.
    neighbors:  Option<HashSet<usize>>, // vertices adjascent to this one.
    ancestors:  Option<Vec<VertexWithConcavity<N>>>, // faces from the original surface.
    uancestors: Option<HashSet<usize>>,
    border:     Option<HashSet<Vector2<usize>>>,
    chull:      Option<TriMesh<Point3<N>>>,
    parts:      Option<Vec<usize>>,
    timestamp:  usize,
    concavity:  N,
    area:       N,
    aabb:       AABB<Point3<N>>
}

impl<N: Real> DualGraphVertex<N> {
    pub fn new(ancestor: usize,
               mesh:     &TriMesh<Point3<N>>,
               raymap:   &HashMap<(u32, u32), usize>) // FIXME: we could get rid of the raymap.
               -> DualGraphVertex<N> {
        let (idx, ns) =
            match mesh.indices {
                IndexBuffer::Unified(ref idx) => (idx[ancestor].clone(), idx[ancestor].clone()),
                IndexBuffer::Split(ref idx) => {
                    let t = idx[ancestor];
                    (Point3::new(t.x.x, t.y.x, t.z.x), Point3::new(t.x.y, t.y.y, t.z.y))
                }
            };

        let triangle = vec!(mesh.coords[idx.x as usize],
                            mesh.coords[idx.y as usize],
                            mesh.coords[idx.z as usize]);

        let area         = utils::triangle_area(&triangle[0], &triangle[1], &triangle[2]);
        let (vmin, vmax) = bounding_volume::point_cloud_aabb(&Id::new(), &triangle[..]);
        let aabb         = AABB::new(vmin, vmax);

        let chull = TriMesh::new(triangle, None, None, None);

        let r1 = raymap.get(&(idx.x, ns.x)).unwrap().clone();
        let r2 = raymap.get(&(idx.y, ns.y)).unwrap().clone();
        let r3 = raymap.get(&(idx.z, ns.z)).unwrap().clone();

        let ancestors = vec!(
            VertexWithConcavity::new(r1, na::zero()),
            VertexWithConcavity::new(r2, na::zero()),
            VertexWithConcavity::new(r3, na::zero())
        );
        let mut uancestors = HashSet::new();
        let _ = uancestors.insert(r1);
        let _ = uancestors.insert(r2);
        let _ = uancestors.insert(r3);

        let mut border = HashSet::new();
        let _ = border.insert(edge(idx.x, idx.y));
        let _ = border.insert(edge(idx.y, idx.z));
        let _ = border.insert(edge(idx.z, idx.x));

        DualGraphVertex {
            neighbors:  Some(HashSet::new()),
            ancestors:  Some(ancestors),
            uancestors: Some(uancestors),
            parts:      Some(vec!(ancestor)),
            border:     Some(border),
            timestamp:  0,
            chull:      Some(chull),
            concavity:  na::zero(),
            area:       area,
            aabb:       aabb
        }
    }

    pub fn hecol(mut edge: DualGraphEdge<N>, graph: &mut [DualGraphVertex<N>]) {
        assert!(edge.exact);

        let valid = edge.v1;
        let other = edge.v2;

        graph[other].ancestors = None;

        let other_neighbors = graph[other].neighbors.take().unwrap();
        let other_parts     = graph[other].parts.take().unwrap();

        /*
         * Merge neighbors.
         */
        for neighbor in other_neighbors.iter() {
            if *neighbor != valid {
                let ga = &mut graph[*neighbor];

                // Replace `other` by `valid`.
                let _ = ga.neighbors.as_mut().unwrap().remove(&other);
                let _ = ga.neighbors.as_mut().unwrap().insert(valid);
            }
        }

        /*
         * Merge ancestors costs.
         */
        let mut new_ancestors = Vec::new();
        let mut last_ancestor: usize = Bounded::max_value();

        loop {
            match edge.ancestors.pop() {
                None => break,
                Some(ancestor) => {
                    // Remove duplicates on the go.
                    if last_ancestor != ancestor.id {
                        last_ancestor = ancestor.id;
                        new_ancestors.push(ancestor)
                    }
                }
            }
        }

        /*
         * Merge ancestors sets.
         */
        let mut other_uancestors = graph[other].uancestors.take().unwrap();
        let mut valid_uancestors = graph[valid].uancestors.take().unwrap();

        // We will push the smallest one to the biggest.
        if other_uancestors.len() > valid_uancestors.len() {
            mem::swap(&mut other_uancestors, &mut valid_uancestors);
        }

        {
            for i in other_uancestors.iter() {
                let _ = valid_uancestors.insert(*i);
            }
        }

        /*
         * Compute the convex hull.
         */
        let valid_chull = graph[valid].chull.take().unwrap();
        let other_chull = graph[other].chull.take().unwrap();

        let mut vtx1 = valid_chull.coords;
        let     vtx2 = other_chull.coords;

        vtx1.extend(vtx2.into_iter());

        // FIXME: use a method to merge convex hulls instead of reconstructing it from scratch.
        let chull = super::convex_hull3(&vtx1[..]);

        /*
         * Merge borders.
         */
        let valid_border = graph[valid].border.take().unwrap();
        let other_border = graph[other].border.take().unwrap();
        let new_border   = valid_border.symmetric_difference(&other_border).map(|e| *e).collect();

        /*
         * Finalize.
         */
        let new_aabb   = graph[valid].aabb.merged(&graph[other].aabb);
        let other_area = graph[other].area;
        let gvalid     = &mut graph[valid];
        gvalid.parts.as_mut().unwrap().extend(other_parts.into_iter());
        gvalid.chull = Some(chull);
        gvalid.aabb = new_aabb;
        gvalid.neighbors.as_mut().unwrap().extend(other_neighbors.into_iter());
        let _ = gvalid.neighbors.as_mut().unwrap().remove(&other);
        let _ = gvalid.neighbors.as_mut().unwrap().remove(&valid);
        gvalid.ancestors = Some(new_ancestors);
        gvalid.area = gvalid.area + other_area;
        gvalid.concavity = edge.concavity;
        gvalid.uancestors = Some(valid_uancestors);
        gvalid.border = Some(new_border);
    }
}

struct DualGraphEdge<N> {
    v1:        usize,
    v2:        usize,
    mcost:     N,
    exact:     bool,
    timestamp: usize,
    ancestors: BinaryHeap<VertexWithConcavity<N>>,
    iv1:       usize,
    iv2:       usize,
    shape:     N,
    concavity: N,
    iray_cast: bool
}

impl<N: Real> DualGraphEdge<N> {
    pub fn new(timestamp:     usize,
               v1:            usize,
               v2:            usize,
               dual_graph:    &[DualGraphVertex<N>],
               coords:        &[Point3<N>],
               max_concavity: N)
               -> DualGraphEdge<N> {
        let mut v1 = v1;
        let mut v2 = v2;

        if v1 > v2 {
            mem::swap(&mut v1, &mut v2);
        }

        let border_1 = dual_graph[v1].border.as_ref().unwrap();
        let border_2 = dual_graph[v2].border.as_ref().unwrap();
        let mut perimeter = na::zero::<N>();

        for d in border_1.symmetric_difference(border_2).map(|e| na::distance(&coords[e.x], &coords[e.y])) {
            perimeter = perimeter + d;
        }

        let area = dual_graph[v1].area + dual_graph[v2].area;

        // FIXME: refactor this.
        let aabb = dual_graph[v1].aabb.merged(&dual_graph[v2].aabb);
        let diagonal = na::distance(aabb.mins(), aabb.maxs());
        let shape_cost;

        if area.is_zero() || diagonal.is_zero() {
            shape_cost = perimeter * perimeter;
        }
        else {
            shape_cost = diagonal * perimeter * perimeter / area;
        }

        let approx_concavity = dual_graph[v1].concavity.max(dual_graph[v2].concavity);

        DualGraphEdge {
            v1:        v1,
            v2:        v2,
            mcost:     -(approx_concavity + max_concavity * shape_cost),
            exact:     false,
            timestamp: timestamp,
            ancestors: BinaryHeap::with_capacity(0),
            iv1:       0,
            iv2:       0,
            shape:     shape_cost,
            concavity: approx_concavity,
            iray_cast: false
        }
    }

    pub fn is_valid(&self, dual_graph: &[DualGraphVertex<N>]) -> bool {
        self.timestamp >= dual_graph[self.v1].timestamp &&
        self.timestamp >= dual_graph[self.v2].timestamp
    }

    pub fn compute_decimation_cost(&mut self,
                                   dual_graph:    &[DualGraphVertex<N>],
                                   rays:          &[Ray<Point3<N>>],
                                   bvt:           &BVT<usize, AABB<Point3<N>>>,
                                   max_cost:      N,
                                   max_concavity: N) {
        assert!(self.is_valid(dual_graph));

        let v1 = &dual_graph[self.v1];
        let v2 = &dual_graph[self.v2];

        /*
         * Estimate the concavity.
         */
        let chull1 = v1.chull.as_ref().unwrap();
        let chull2 = v2.chull.as_ref().unwrap();
        let chull  = ConvexPair::new(&chull1.coords[..], &chull2.coords[..]);
        let _max: N = Bounded::max_value();

        let a1 = v1.ancestors.as_ref().unwrap();
        let a2 = v2.ancestors.as_ref().unwrap();

        let max_concavity = max_concavity.min(max_cost - max_concavity * self.shape);

        fn cast_ray<'a, N: Real>(chull:      &ConvexPair<'a, N>,
                                 ray:        &Ray<Point3<N>>,
                                 id:         usize,
                                 concavity:  &mut N,
                                 ancestors:  &mut BinaryHeap<VertexWithConcavity<N>>) {
            let sv   = chull.support_point(&Id::new(), &ray.dir);
            let distance = na::dot(&sv.coords, &ray.dir);

            if !relative_eq!(distance, na::zero()) {
                let shift: N = na::convert(0.1f64);
                let outside_point = ray.origin + ray.dir * (distance + shift);

                match chull.toi_with_ray(&Id::new(), &Ray::new(outside_point, -ray.dir), true) {
                    None      => {
                        ancestors.push(VertexWithConcavity::new(id, na::zero()))
                    },
                    Some(toi) => {
                        let new_concavity = distance + shift - toi;
                        ancestors.push(VertexWithConcavity::new(id, new_concavity));
                        if *concavity < new_concavity {
                            *concavity = new_concavity
                        }
                    }
                }
            }
            else {
                ancestors.push(VertexWithConcavity::new(id, na::zero()));
            }
        }

        // Start with the rays we know.
        while (self.iv1 != a1.len() || self.iv2 != a2.len()) && self.concavity <= max_concavity {
            let id;

            if self.iv1 != a1.len() && (self.iv2 == a2.len() || a1[self.iv1].concavity > a2[self.iv2].concavity) {
                id = a1[self.iv1].id;
                self.iv1 = self.iv1 + 1;
            }
            else {
                id = a2[self.iv2].id;
                self.iv2 = self.iv2 + 1;
            }

            let ray = &rays[id].clone();

            if ray.dir.is_zero() { // the ray was set to zero if it was invalid.
                continue;
            }

            cast_ray(&chull, ray, id, &mut self.concavity, &mut self.ancestors);
        }

        // FIXME: (optimization) find a way to stop the cast if we exceed the max concavity.
        if !self.iray_cast && self.iv1 == a1.len() && self.iv2 == a2.len() {
            self.iray_cast = true;
            let mut internal_rays = Vec::new();

            {
                let aabb = v1.aabb.merged(&v2.aabb);
                let mut visitor = BoundingVolumeInterferencesCollector::new(&aabb, &mut internal_rays);

                bvt.visit(&mut visitor);
            }

            let uancestors_v1 = v1.uancestors.as_ref().unwrap();
            let uancestors_v2 = v2.uancestors.as_ref().unwrap();

            for ray_id in internal_rays.iter() {
                let ray = &rays[*ray_id];

                if !uancestors_v1.contains(ray_id) && !uancestors_v2.contains(ray_id) {
                    // XXX: we could just remove the zero-dir rays from the ancestors list!
                    if ray.dir.is_zero() { // The ray was set to zero if it was invalid.
                        continue;
                    }

                    // We determine if the point is inside of the convex hull or not.
                    // XXX: use a point-in-implicit test instead of a ray-cast!
                    match chull.toi_with_ray(&Id::new(), ray, true) {
                        None        => continue,
                        Some(inter) => {
                            if inter.is_zero() {
                                // ok, the point is inside, performe the real ray-cast.
                                cast_ray(&chull, ray, *ray_id, &mut self.concavity, &mut self.ancestors);
                            }
                        }
                    }
                }
            }
        }

        self.mcost = -(self.concavity + max_concavity * self.shape);
        self.exact = self.iv1 == a1.len() && self.iv2 == a2.len();
    }
}

impl<N: PartialEq> PartialEq for DualGraphEdge<N> {
    #[inline]
    fn eq(&self, other: &DualGraphEdge<N>) -> bool {
        self.mcost.eq(&other.mcost)
    }
}

impl<N: PartialEq> Eq for DualGraphEdge<N> {
}

impl<N: PartialOrd> PartialOrd for DualGraphEdge<N> {
    #[inline]
    fn partial_cmp(&self, other: &DualGraphEdge<N>) -> Option<Ordering> {
        self.mcost.partial_cmp(&other.mcost)
    }
}

impl<N: PartialOrd> Ord for DualGraphEdge<N> {
    #[inline]
    fn cmp(&self, other: &DualGraphEdge<N>) -> Ordering {
        if self.mcost < other.mcost {
            Ordering::Less
        }
        else if self.mcost > other.mcost {
            Ordering::Greater
        }
        else {
            Ordering::Equal
        }
    }
}

struct VertexWithConcavity<N> {
    id:        usize,
    concavity: N
}

impl<N> VertexWithConcavity<N> {
    #[inline]
    pub fn new(id: usize, concavity: N) -> VertexWithConcavity<N> {
        VertexWithConcavity {
            id:        id,
            concavity: concavity
        }
    }
}

impl<N: PartialEq> PartialEq for VertexWithConcavity<N> {
    #[inline]
    fn eq(&self, other: &VertexWithConcavity<N>) -> bool {
        self.concavity == other.concavity && self.id == other.id
    }
}

impl<N: PartialEq> Eq for VertexWithConcavity<N> {
}

impl<N: PartialOrd> PartialOrd for VertexWithConcavity<N> {
    #[inline]
    fn partial_cmp(&self, other: &VertexWithConcavity<N>) -> Option<Ordering> {
        if self.concavity < other.concavity {
            Some(Ordering::Less)
        }
        else if self.concavity > other.concavity {
            Some(Ordering::Greater)
        }
        else {
            Some(self.id.cmp(&other.id))
        }
    }
}

impl<N: PartialOrd> Ord for VertexWithConcavity<N> {
    #[inline]
    fn cmp(&self, other: &VertexWithConcavity<N>) -> Ordering {
        if self.concavity < other.concavity {
            Ordering::Less
        }
        else if self.concavity > other.concavity {
            Ordering::Greater
        }
        else {
            self.id.cmp(&other.id)
        }
    }
}

#[inline]
fn edge(a: u32, b: u32) -> Vector2<usize> {
    if a > b {
        Vector2::new(b as usize, a as usize)
    }
    else {
        Vector2::new(a as usize, b as usize)
    }
}

fn compute_ray_bvt<N: Real>(rays: &[Ray<Point3<N>>]) -> BVT<usize, AABB<Point3<N>>> {
    let aabbs = rays.iter().enumerate().map(|(i, r)| (i, AABB::new(r.origin, r.origin))).collect();

    BVT::new_balanced(aabbs)
}

fn compute_rays<N: Real>(mesh: &TriMesh<Point3<N>>) -> (Vec<Ray<Point3<N>>>, HashMap<(u32, u32), usize>) {
    let mut rays   = Vec::new();
    let mut raymap = HashMap::new();

    {
        let coords  = &mesh.coords[..];
        let normals = &mesh.normals.as_ref().unwrap()[..];

        let mut add_ray = |coord: u32, normal: u32| {
            let key = (coord, normal);
            let existing = match raymap.entry(key) {
                Entry::Occupied(entry) => entry.into_mut(),
                Entry::Vacant(entry)   => entry.insert(rays.len())
            };

            if *existing == rays.len() {
                let coord = coords[coord as usize].clone();
                let mut normal = normals[normal as usize].clone();

                if normal.normalize_mut().is_zero() {
                    normal = na::zero();
                }

                rays.push(Ray::new(coord, normal));
            }
        };

        match mesh.indices {
            IndexBuffer::Unified(ref b) => {
                for t in b.iter() {
                    add_ray(t.x, t.x);
                    add_ray(t.y, t.y);
                    add_ray(t.z, t.z);
                }
            },
            IndexBuffer::Split(ref b) => {
                for t in b.iter() {
                    add_ray(t.x.x, t.x.y);
                    add_ray(t.y.x, t.y.y);
                    add_ray(t.z.x, t.z.y);
                }
            }
        }
    }

    (rays, raymap)
}


fn compute_dual_graph<N: Real>(mesh:   &TriMesh<Point3<N>>,
                               raymap: &HashMap<(u32, u32), usize>)
                               -> Vec<DualGraphVertex<N>> {
    // XXX Loss of determinism because of the randomized HashMap.
    let mut prim_edges = HashMap::new();
    let mut dual_vertices: Vec<DualGraphVertex<N>> =
        (0 .. mesh.num_triangles()).map(|i| DualGraphVertex::new(i, mesh, raymap)).collect();

    {
        let mut add_triangle_edges = Box::new(|i: usize, t: &Point3<u32>| {
            let es = [ edge(t.x, t.y), edge(t.y, t.z), edge(t.z, t.x) ];

            for e in es.iter() {
                let other = match prim_edges.entry(*e) {
                    Entry::Occupied(entry) => entry.into_mut(),
                    Entry::Vacant(entry)   => entry.insert(i)
                };

                if *other != i {
                    // register the adjascency.
                    let _ = dual_vertices[i].neighbors.as_mut().unwrap().insert(*other);
                    let _ = dual_vertices[*other].neighbors.as_mut().unwrap().insert(i);
                }
            }
        });

        match mesh.indices {
            IndexBuffer::Unified(ref b) => {
                for (i, t) in b.iter().enumerate() {
                    add_triangle_edges(i, t)
                }
            },
            IndexBuffer::Split(ref b) => {
                for (i, t) in b.iter().enumerate() {
                    let t_idx = Point3::new(t.x.x, t.y.x, t.z.x);
                    add_triangle_edges(i, &t_idx)
                }
            }
        }
    }

    dual_vertices
}

struct ConvexPair<'a, N: 'a + Real> {
    a: &'a [Point3<N>],
    b: &'a [Point3<N>]
}

impl<'a, N: Real> ConvexPair<'a, N> {
    pub fn new(a: &'a [Point3<N>], b: &'a [Point3<N>]) -> ConvexPair<'a, N> {
        ConvexPair {
            a: a,
            b: b
        }
    }
}

impl<'a, N: Real> SupportMap<Point3<N>, Id> for ConvexPair<'a, N> {
    #[inline]
    fn support_point(&self, _: &Id, dir: &Vector3<N>) -> Point3<N> {
        let sa = utils::point_cloud_support_point(dir, self.a);
        let sb = utils::point_cloud_support_point(dir, self.b);

        if na::dot(&sa.coords, dir) > na::dot(&sb.coords, dir) {
            sa
        }
        else {
            sb
        }
    }
}

impl<'a, N: Real> RayCast<Point3<N>, Id> for ConvexPair<'a, N> {
    #[inline]
    fn toi_and_normal_with_ray(&self, id: &Id, ray: &Ray<Point3<N>>, solid: bool) -> Option<RayIntersection<Vector3<N>>> {
        ray_internal::implicit_toi_and_normal_with_ray(
            id,
            self,
            &mut JohnsonSimplex::<Point3<N>>::new_w_tls(),
            ray,
            solid)
    }
}
