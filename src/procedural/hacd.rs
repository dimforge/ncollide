use std::num::Zero;
use std::iter::AdditiveIterator;
use std::mem;
use std::collections::{HashMap, HashSet, PriorityQueue};
use std::collections::hashmap::{Occupied, Vacant};
use std::rand::{IsaacRng, Rng};
use std::num::Bounded;
use std::hash::sip::SipHasher;
use na;
use na::{Pnt3, Vec2, Vec3, Identity, Iterable, Norm};
use narrow::algorithm::johnson_simplex::JohnsonSimplex;
use math::{Scalar, Point, Vect};
use implicit::Implicit;
use ray::{Ray, RayCast, RayIntersection};
use ray;
use procedural::{TriMesh, SplitIndexBuffer, UnifiedIndexBuffer};
use utils;
use bounding_volume::{AABB, BoundingVolume};
use bounding_volume;
use geom::Convex;
use partitioning::{BVT, BoundingVolumeInterferencesCollector};

/// Approximate convex decomposition of a triangle mesh.
#[cfg(feature = "3d")]
pub fn hacd(mesh:           TriMesh<Scalar, Pnt3<Scalar>, Vec3<Scalar>>,
            error:          Scalar,
            min_components: uint)
            -> (Vec<Convex>, Vec<Vec<uint>>) {
    assert!(mesh.normals.is_some(), "Vertex normals are required to compute the convex decomposition.");

    let mut mesh = mesh;

    let mut edges      = PriorityQueue::new();
    let (center, diag) = normalize(&mut mesh);
    let (rays, raymap) = compute_rays(&mesh);
    let mut dual_graph = compute_dual_graph(&mesh, &raymap);
    let bvt            = compute_ray_bvt(rays.as_slice());

    /*
     * Initialize the priority queue.
     */
    for (i, v) in dual_graph.iter().enumerate() {
        for n in v.neighbors.as_ref().unwrap().iter() {
            if i < *n {
                let edge = DualGraphEdge::new(0, i, *n, dual_graph.as_slice(), mesh.coords.as_slice(), error);

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
                if !top.is_valid(dual_graph.as_slice()) {
                    continue; // this edge has been invalidated.
                }

                if !top.exact {
                    // the cost is just an upper bound.
                    let _max: Scalar = Bounded::max_value();
                    let mut top_cost = -_max;

                    loop {
                        let remove = match edges.top() {
                            None        => false,
                            Some(ref e) => {
                                if !top.is_valid(dual_graph.as_slice()) {
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
                    top.compute_decimation_cost(dual_graph.as_slice(), rays.as_slice(), &bvt, -top_cost, error);

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
                top.compute_decimation_cost(dual_graph.as_slice(), rays.as_slice(), &bvt, Bounded::max_value(), error);

                if top.concavity > error {
                    continue;
                }

                curr_time = curr_time + 1;

                let v1 = top.v1;
                let v2 = top.v2;

                DualGraphVertex::hecol(top, dual_graph.as_mut_slice());

                assert!(dual_graph[v1].timestamp != Bounded::max_value());
                assert!(dual_graph[v2].timestamp != Bounded::max_value());
                dual_graph.get_mut(v1).timestamp = curr_time;
                dual_graph.get_mut(v2).timestamp = Bounded::max_value(); // Mark as invalid.

                v1
            }
        };

        for n in dual_graph[to_add].neighbors.as_ref().unwrap().iter() {
            let edge = DualGraphEdge::new(curr_time, to_add, *n, dual_graph.as_slice(), mesh.coords.as_slice(), error);

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

            denormalize(chull.mesh_mut(), &center, &diag);
            result.push(chull);
            parts.push(vertex.parts.unwrap());
        }
    }

    (result, parts)
}

#[cfg(feature = "3d")]
fn normalize(mesh: &mut TriMesh<Scalar, Point, Vect>) -> (Point, Scalar) {
    let (mins, maxs) = bounding_volume::point_cloud_aabb(&Identity::new(), mesh.coords.as_slice());
    let diag   = na::dist(&mins, &maxs);
    let center = na::center(&mins, &maxs);

    mesh.translate_by(&(-*center.as_vec()));
    let _1: Scalar = na::one();
    mesh.scale_by_scalar(&(_1 / diag));

    (center, diag)
}

#[cfg(feature = "3d")]
fn denormalize(mesh: &mut TriMesh<Scalar, Pnt3<Scalar>, Vec3<Scalar>>, center: &Point, diag: &Scalar) {
    mesh.scale_by_scalar(diag);
    mesh.translate_by(center.as_vec());
}

#[cfg(feature = "3d")]
struct DualGraphVertex {
    neighbors:  Option<HashSet<uint, SipHasher>>, // vertices adjascent to this one.
    ancestors:  Option<Vec<VertexWithConcavity>>, // faces from the original surface.
    uancestors: Option<HashSet<uint, SipHasher>>,
    border:     Option<HashSet<Vec2<uint>>>,
    chull:      Option<Convex>,
    parts:      Option<Vec<uint>>,
    timestamp:  uint,
    concavity:  Scalar,
    area:       Scalar,
    aabb:       AABB
}

#[cfg(feature = "3d")]
impl DualGraphVertex {
    pub fn new(ancestor: uint,
               mesh:     &TriMesh<Scalar, Pnt3<Scalar>, Vec3<Scalar>>,
               raymap:   &HashMap<(u32, u32), uint>) // FIXME: we could get rid of the raymap.
               -> DualGraphVertex {
        let (idx, ns) =
            match mesh.indices {
                UnifiedIndexBuffer(ref idx) => (idx[ancestor].clone(), idx[ancestor].clone()),
                SplitIndexBuffer(ref idx) => {
                    let t = idx[ancestor];
                    (Vec3::new(t.x.x, t.y.x, t.z.x), Vec3::new(t.x.y, t.y.y, t.z.y))
                }
            };

        let triangle = vec!(mesh.coords[idx.x as uint],
                            mesh.coords[idx.y as uint],
                            mesh.coords[idx.z as uint]);

        let area         = utils::triangle_area(&triangle[0], &triangle[1], &triangle[2]);
        let (vmin, vmax) = bounding_volume::point_cloud_aabb(&Identity::new(), triangle.as_slice());
        let aabb         = AABB::new(vmin, vmax);

        let chull = unsafe {;
            Convex::new_with_convex_mesh(TriMesh::new(triangle, None, None, None))
        };

        let r1 = raymap.find(&(idx.x, ns.x)).unwrap().clone();
        let r2 = raymap.find(&(idx.y, ns.y)).unwrap().clone();
        let r3 = raymap.find(&(idx.z, ns.z)).unwrap().clone();

        let mut rng   = IsaacRng::new_unseeded();
        let ancestors = vec!(
            VertexWithConcavity::new(r1, na::zero()),
            VertexWithConcavity::new(r2, na::zero()),
            VertexWithConcavity::new(r3, na::zero())
        );
        let mut uancestors = HashSet::with_hasher(SipHasher::new_with_keys(rng.gen(), rng.gen()));
        uancestors.insert(r1);
        uancestors.insert(r2);
        uancestors.insert(r3);

        let mut border = HashSet::new();
        border.insert(edge(idx.x, idx.y));
        border.insert(edge(idx.y, idx.z));
        border.insert(edge(idx.z, idx.x));

        DualGraphVertex {
            neighbors:  Some(HashSet::with_hasher(SipHasher::new_with_keys(rng.gen(), rng.gen()))),
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

    pub fn hecol(mut edge: DualGraphEdge, graph: &mut [DualGraphVertex]) {
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
                ga.neighbors.as_mut().unwrap().remove(&other);
                ga.neighbors.as_mut().unwrap().insert(valid);
            }
        }

        /*
         * Merge ancestors costs.
         */
        let mut new_ancestors = Vec::new();
        let mut last_ancestor: uint = Bounded::max_value();

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
                valid_uancestors.insert(*i);
            }
        }

        /*
         * Compute the convex hull.
         */
        let valid_chull = graph[valid].chull.take().unwrap();
        let other_chull = graph[other].chull.take().unwrap();

        let mut vtx1 = valid_chull.unwrap().coords;
        let     vtx2 = other_chull.unwrap().coords;

        vtx1.extend(vtx2.into_iter());

        // FIXME: use a method to merge convex hulls instead of reconstructing it from scratch.
        let chull = Convex::new(vtx1.as_slice());

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
        gvalid.neighbors.as_mut().unwrap().remove(&other);
        gvalid.neighbors.as_mut().unwrap().remove(&valid);
        gvalid.ancestors = Some(new_ancestors);
        gvalid.area = gvalid.area + other_area;
        gvalid.concavity = edge.concavity;
        gvalid.uancestors = Some(valid_uancestors);
        gvalid.border = Some(new_border);
    }
}

#[cfg(feature = "3d")]
struct DualGraphEdge {
    v1:        uint,
    v2:        uint,
    mcost:     Scalar,
    exact:     bool,
    timestamp: uint,
    ancestors: PriorityQueue<VertexWithConcavity>,
    iv1:       uint,
    iv2:       uint,
    shape:     Scalar,
    concavity: Scalar,
    iray_cast: bool
}

#[cfg(feature = "3d")]
impl DualGraphEdge {
    pub fn new(timestamp:     uint,
               v1:            uint,
               v2:            uint,
               dual_graph:    &[DualGraphVertex],
               coords:        &[Point],
               max_concavity: Scalar)
               -> DualGraphEdge {
        let mut v1 = v1;
        let mut v2 = v2;

        if v1 > v2 {
            mem::swap(&mut v1, &mut v2);
        }

        let border_1 = dual_graph[v1].border.as_ref().unwrap();
        let border_2 = dual_graph[v2].border.as_ref().unwrap();
        let perimeter = border_1.symmetric_difference(border_2).map(|e| na::dist(&coords[e.x], &coords[e.y])).sum();
        let area = dual_graph[v1].area + dual_graph[v2].area;

        // FIXME: refactor this.
        let aabb = dual_graph[v1].aabb.merged(&dual_graph[v2].aabb);
        let diagonal = na::dist(aabb.mins(), aabb.maxs());
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
            ancestors: PriorityQueue::with_capacity(0),
            iv1:       0,
            iv2:       0,
            shape:     shape_cost,
            concavity: approx_concavity,
            iray_cast: false
        }
    }

    pub fn is_valid(&self, dual_graph: &[DualGraphVertex]) -> bool {
        self.timestamp >= dual_graph[self.v1].timestamp &&
        self.timestamp >= dual_graph[self.v2].timestamp
    }

    pub fn compute_decimation_cost(&mut self,
                                   dual_graph:    &[DualGraphVertex],
                                   rays:          &[Ray],
                                   bvt:           &BVT<uint, AABB>,
                                   max_cost:      Scalar,
                                   max_concavity: Scalar) {
        assert!(self.is_valid(dual_graph));

        let v1 = &dual_graph[self.v1];
        let v2 = &dual_graph[self.v2];

        /*
         * Estimate the concavity.
         */
        let chull1 = v1.chull.as_ref().unwrap();
        let chull2 = v2.chull.as_ref().unwrap();
        let chull  = ConvexPair::new(chull1, chull2);
        let _max: Scalar = Bounded::max_value();

        let a1 = v1.ancestors.as_ref().unwrap();
        let a2 = v2.ancestors.as_ref().unwrap();

        let max_concavity = max_concavity.min(max_cost - max_concavity * self.shape);

        fn cast_ray<'a>(chull:      &ConvexPair<'a>,
                        ray:        &Ray,
                        id:         uint,
                        concavity:  &mut Scalar,
                        ancestors:  &mut PriorityQueue<VertexWithConcavity>) {
            let sv   = chull.support_point(&Identity::new(), &ray.dir);
            let dist = na::dot(sv.as_vec(), &ray.dir);

            if !na::approx_eq(&dist, &na::zero()) {
                let shift: Scalar = na::cast(0.1f64);
                let outside_point = ray.orig + ray.dir * (dist + shift);

                match chull.toi_with_ray(&Ray::new(outside_point, -ray.dir), true) {
                    None      => {
                        ancestors.push(VertexWithConcavity::new(id, na::zero()))
                    },
                    Some(toi) => {
                        let new_concavity = dist + shift - toi;
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
                    match chull.toi_with_ray(ray, true) {
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

#[cfg(feature = "3d")]
impl PartialEq for DualGraphEdge {
    #[inline]
    fn eq(&self, other: &DualGraphEdge) -> bool {
        self.mcost.eq(&other.mcost)
    }
}

#[cfg(feature = "3d")]
impl Eq for DualGraphEdge {
}

#[cfg(feature = "3d")]
impl PartialOrd for DualGraphEdge {
    #[inline]
    fn partial_cmp(&self, other: &DualGraphEdge) -> Option<Ordering> {
        self.mcost.partial_cmp(&other.mcost)
    }
}

#[cfg(feature = "3d")]
impl Ord for DualGraphEdge {
    #[inline]
    fn cmp(&self, other: &DualGraphEdge) -> Ordering {
        if self.mcost < other.mcost {
            Less
        }
        else if self.mcost > other.mcost {
            Greater
        }
        else {
            Equal
        }
    }
}

#[cfg(feature = "3d")]
struct VertexWithConcavity {
    id:        uint,
    concavity: Scalar
}

#[cfg(feature = "3d")]
impl VertexWithConcavity {
    #[inline]
    pub fn new(id: uint, concavity: Scalar) -> VertexWithConcavity {
        VertexWithConcavity {
            id:        id,
            concavity: concavity
        }
    }
}

#[cfg(feature = "3d")]
impl PartialEq for VertexWithConcavity {
    #[inline]
    fn eq(&self, other: &VertexWithConcavity) -> bool {
        self.concavity == other.concavity && self.id == other.id
    }
}

#[cfg(feature = "3d")]
impl Eq for VertexWithConcavity {
}

#[cfg(feature = "3d")]
impl PartialOrd for VertexWithConcavity {
    #[inline]
    fn partial_cmp(&self, other: &VertexWithConcavity) -> Option<Ordering> {
        if self.concavity < other.concavity {
            Some(Less)
        }
        else if self.concavity > other.concavity {
            Some(Greater)
        }
        else {
            Some(self.id.cmp(&other.id))
        }
    }
}

#[cfg(feature = "3d")]
impl Ord for VertexWithConcavity {
    #[inline]
    fn cmp(&self, other: &VertexWithConcavity) -> Ordering {
        if self.concavity < other.concavity {
            Less
        }
        else if self.concavity > other.concavity {
            Greater
        }
        else {
            self.id.cmp(&other.id)
        }
    }
}

#[cfg(feature = "3d")]
#[inline]
fn edge(a: u32, b: u32) -> Vec2<uint> {
    if a > b {
        Vec2::new(b as uint, a as uint)
    }
    else {
        Vec2::new(a as uint, b as uint)
    }
}

#[cfg(feature = "3d")]
fn compute_ray_bvt(rays: &[Ray]) -> BVT<uint, AABB> {
    let aabbs = rays.iter().enumerate().map(|(i, r)| (i, AABB::new(r.orig, r.orig))).collect();

    BVT::new_balanced(aabbs)
}

#[cfg(feature = "3d")]
fn compute_rays(mesh: &TriMesh<Scalar, Pnt3<Scalar>, Vec3<Scalar>>) -> (Vec<Ray>, HashMap<(u32, u32), uint>) {
    let mut rays   = Vec::new();
    let mut raymap = HashMap::new();

    {
        let coords  = mesh.coords.as_slice();
        let normals = mesh.normals.as_ref().unwrap().as_slice();

        let add_ray = |coord: u32, normal: u32| {
            let existing = match raymap.entry((coord, normal)) {
                Occupied(entry) => entry.into_mut(),
                Vacant(entry)   => entry.set(rays.len())
            };

            if *existing == rays.len() {
                let coord = coords[coord as uint].clone();
                let mut normal = normals[normal as uint].clone();

                if normal.normalize().is_zero() {
                    normal = na::zero();
                }

                rays.push(Ray::new(coord, normal));
            }
        };

        match mesh.indices {
            UnifiedIndexBuffer(ref b) => {
                for t in b.iter() {
                    add_ray(t.x, t.x);
                    add_ray(t.y, t.y);
                    add_ray(t.z, t.z);
                }
            },
            SplitIndexBuffer(ref b) => {
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


#[cfg(feature = "3d")]
fn compute_dual_graph(mesh:   &TriMesh<Scalar, Pnt3<Scalar>, Vec3<Scalar>>,
                      raymap: &HashMap<(u32, u32), uint>)
                      -> Vec<DualGraphVertex> {
    let mut rng           = IsaacRng::new_unseeded();
    let mut prim_edges    = HashMap::with_hasher(SipHasher::new_with_keys(rng.gen(), rng.gen()));
    let mut dual_vertices = Vec::from_fn(mesh.num_triangles(), |i| DualGraphVertex::new(i, mesh, raymap));

    {
        let add_triangle_edges = |i: uint, t: &Vec3<u32>| {
            let es = [ edge(t.x, t.y), edge(t.y, t.z), edge(t.z, t.x) ];

            for e in es.iter() {
                let other = match prim_edges.entry(e.clone()) {
                    Occupied(entry) => entry.into_mut(),
                    Vacant(entry)   => entry.set(i)
                };

                if *other != i {
                    // register the adjascency.
                    dual_vertices.get_mut(i).neighbors.as_mut().unwrap().insert(*other);
                    dual_vertices.get_mut(*other).neighbors.as_mut().unwrap().insert(i);
                }
            }
        };

        match mesh.indices {
            UnifiedIndexBuffer(ref b) => {
                for (i, t) in b.iter().enumerate() {
                    add_triangle_edges(i, t)
                }
            },
            SplitIndexBuffer(ref b) => {
                for (i, t) in b.iter().enumerate() {
                    let t_idx = Vec3::new(t.x.x, t.y.x, t.z.x);
                    add_triangle_edges(i, &t_idx)
                }
            }
        }
    }

    dual_vertices
}

struct ConvexPair<'a> {
    a: &'a Convex,
    b: &'a Convex
}

impl<'a> ConvexPair<'a> {
    pub fn new(a: &'a Convex, b: &'a Convex) -> ConvexPair<'a> {
        ConvexPair {
            a: a,
            b: b
        }
    }
}

impl<'a> Implicit<Point, Vect, Identity> for ConvexPair<'a> {
    fn support_point(&self, transform: &Identity, dir: &Vect) -> Point {
        let sa = self.a.support_point(transform, dir);
        let sb = self.b.support_point(transform, dir);

        if na::dot(sa.as_vec(), dir) > na::dot(sb.as_vec(), dir) {
            sa
        }
        else {
            sb
        }
    }
}

impl<'a> RayCast for ConvexPair<'a> {
    fn toi_and_normal_with_ray(&self, ray: &Ray, solid: bool) -> Option<RayIntersection> {
        ray::implicit_toi_and_normal_with_ray(
            &Identity::new(),
            self,
            &mut JohnsonSimplex::<Point, Vect>::new_w_tls(),
            ray,
            solid)
    }
}
