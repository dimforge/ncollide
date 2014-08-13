use std::num::Zero;
use std::mem;
use std::collections::{HashMap, HashSet, PriorityQueue};
use std::rand::{IsaacRng, Rng};
use std::num::Bounded;
use std::hash::sip::SipHasher;
use nalgebra::na;
use nalgebra::na::{Vec2, Vec3, Identity, Iterable, Norm};
use narrow::algorithm::johnson_simplex::JohnsonSimplex;
use math::Scalar;
use implicit::{HasMargin, Implicit};
use implicit;
use ray::{Ray, RayCast, RayIntersection};
use ray;
use procedural::{Polyline, TriMesh, SplitIndexBuffer, UnifiedIndexBuffer};
use procedural;
use utils;
use bounding_volume;
use geom::Convex;

/// Not implemented.
#[dim4]
#[doc(hidden)]
pub fn hacd(mesh: &TriMesh<Scalar, Vec3<Scalar>>, error: Scalar) -> Vec<TriMesh<Scalar, Vec3<Scalar>>> {
    fail!("Not yet implemented.")
}

/// Approximate convex decomposition of a polyline.
#[dim2]
pub fn hacd(mesh: &Polyline<Scalar, Vec2<Scalar>>, error: Scalar) -> Vec<Polyline<Scalar, Vec2<Scalar>>> {
    fail!("Not yet implemented.")
}

/// Approximate convex decomposition of a triangle mesh.
#[dim3]
pub fn hacd(mesh:                &TriMesh<Scalar, Vec3<Scalar>>,
            error:               Scalar,
            min_components:      uint,
            max_vertex_per_hull: uint)
            -> Vec<Convex> {
    assert!(mesh.normals.is_some(), "Vertex normals are required to compute the convex decomposition.");

    let mut edges      = PriorityQueue::new();
    let (rays, raymap) = compute_rays(mesh);
    let mut dual_graph = compute_dual_graph(mesh, &raymap);

    /*
     * Initialize the priority queue.
     */
    if max_vertex_per_hull >= 4 {
        for (i, v) in dual_graph.iter().enumerate() {
            for n in v.neighbors.as_ref().expect("Internal error: 1").iter() {
                if i < *n {
                    let edge = DualGraphEdge::new(0, i, *n, dual_graph.as_slice());

                    edges.push(edge);
                }
            }
        }
    }

    // println!("Number of edges: {}", edges.len());

    /*
     * Decimation.
     */
    let mut curr_time = 0;
    let mut useful_optim = 0u;
    let mut exact_computations = 0u;
    let mut rejected_exact = 0u;
    let mut pushedback_exact = 0u;
    let mut accepted_exact = 0u;
    let mut rerun_casts = 0u;

    while dual_graph.len() - curr_time > min_components {
        let to_add = match edges.pop() {
            None          => break,
            Some(mut top) => {
                if !top.is_valid(dual_graph.as_slice()) {
                    if !top.exact {
                        useful_optim  = useful_optim + 1;
                    }
                    else {
                        rejected_exact = rejected_exact + 1;
                    }
                    continue; // this edge has been invalidated.
                }

                if !top.exact {
                    // the cost is just an upper bound.
                    let _M: Scalar = Bounded::max_value();
                    let mut top_cost = -_M;

                    loop {
                        let remove = match edges.top() {
                            None        => false,
                            Some(ref e) => {
                                if !top.is_valid(dual_graph.as_slice()) {
                                    if !e.exact {
                                        useful_optim = useful_optim + 1;
                                    }
                                    else {
                                        rejected_exact = rejected_exact + 1;
                                    }
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

                    if top.chull.is_some() {
                        rerun_casts = rerun_casts + 1;
                    }

                    // Compute a bounded decimation cost.
                    top.compute_decimation_cost(dual_graph.as_slice(), rays.as_slice(), -top_cost);

                    exact_computations = exact_computations + 1;

                    if top.mcost < top_cost {
                        // we are not the greatest cost any more.
                        edges.push(top);
                        pushedback_exact = pushedback_exact + 1;
                        continue;
                    }
                }

                // Ensure the exact decimation cost has been computed.
                top.compute_decimation_cost(dual_graph.as_slice(), rays.as_slice(), Bounded::max_value());

                if -top.mcost > error {
                    break;
                }

                accepted_exact = accepted_exact + 1;

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

        let to_add_nvtx = dual_graph[to_add].chull.as_ref().unwrap().mesh().coords.len();
        for n in dual_graph[to_add].neighbors.as_ref().expect("Internal error: 4").iter() {
            let other_nvtx = dual_graph[to_add].chull.as_ref().unwrap().mesh().coords.len();

            if to_add_nvtx + other_nvtx <= max_vertex_per_hull {
                let edge = DualGraphEdge::new(curr_time, to_add, *n, dual_graph.as_slice());

                edges.push(edge);
            }
        }
    }

    println!("Number of useful optims: {}", useful_optim);
    println!("Number of exact computations: {}", exact_computations);
    println!("Number of rejected exact: {}", rejected_exact);
    println!("Number of pushed back exact: {}", pushedback_exact);
    println!("Number of accepted exact: {}", accepted_exact);
    println!("Number of rerun casts: {}", rerun_casts);
    println!("Remaining edges: {}", edges.len());

    /*
     * Build the decomposition from the remaining vertices.
     */
    let mut result = Vec::with_capacity(dual_graph.len() - curr_time);

    for vertex in dual_graph.move_iter() {
        if vertex.timestamp != Bounded::max_value() {
            result.push(vertex.chull.expect("Internal error: 5"))
        }
    }

    result
}

#[dim3]
struct DualGraphVertex {
    neighbors: Option<HashSet<uint, SipHasher>>, // vertices adjascent to this one.
    ancestors: Option<Vec<VertexWithConcavity>>, // faces from the original surface.
    timestamp: uint,
    chull:     Option<Convex>,
    mcost:     Scalar
}

#[dim3]
impl DualGraphVertex {
    pub fn new(ancestor: uint,
               mesh:     &TriMesh<Scalar, Vec3<Scalar>>,
               raymap:   &HashMap<(u32, u32), uint>) -> DualGraphVertex {
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

        let chull = unsafe {;
            Convex::new_with_convex_mesh(TriMesh::new(triangle, None, None, None), na::zero())
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

        DualGraphVertex {
            neighbors: Some(HashSet::with_hasher(SipHasher::new_with_keys(rng.gen(), rng.gen()))),
            ancestors: Some(ancestors),
            timestamp: 0,
            chull:     Some(chull),
            mcost:     na::zero()
        }
    }

    pub fn hecol(mut edge: DualGraphEdge, graph: &mut [DualGraphVertex]) {
        let valid = edge.v1;
        let other = edge.v2;

        graph[valid].chull     = None;
        graph[other].chull     = None;
        graph[other].ancestors = None;

        let other_neighbors = graph[other].neighbors.take_unwrap();

        for neighbor in other_neighbors.iter() {
            if *neighbor != valid {
                let ga = &mut graph[*neighbor];

                // Replace `other` by `valid`.
                ga.neighbors.as_mut().expect("Internal error: 6").remove(&other);
                ga.neighbors.as_mut().expect("Internal error: 7").insert(valid);
            }
        }

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

        let gvalid = &mut graph[valid];
        gvalid.chull = Some(*edge.chull.expect("Internal error: 8"));
        gvalid.neighbors.as_mut().expect("Internal error: 9").extend(other_neighbors.move_iter());
        gvalid.neighbors.as_mut().expect("Internal error: 10").remove(&other);
        gvalid.neighbors.as_mut().expect("Internal error: 11").remove(&valid);
        gvalid.ancestors = Some(new_ancestors);
        gvalid.mcost = edge.mcost;
    }
}

#[dim3]
struct DualGraphEdge {
    v1:        uint,
    v2:        uint,
    mcost:     Scalar,
    exact:     bool,
    timestamp: uint,
    chull:     Option<Box<Convex>>,
    ancestors: PriorityQueue<VertexWithConcavity>,
    iv1:       uint,
    iv2:       uint
}

#[dim3]
impl DualGraphEdge {
    pub fn new(timestamp:  uint,
               v1:         uint,
               v2:         uint,
               dual_graph: &[DualGraphVertex])
               -> DualGraphEdge {
        let mut v1 = v1;
        let mut v2 = v2;

        if v1 > v2 {
            mem::swap(&mut v1, &mut v2);
        }

        DualGraphEdge {
            v1:        v1,
            v2:        v2,
            mcost:     dual_graph[v1].mcost.min(dual_graph[v2].mcost),
            exact:     false,
            timestamp: timestamp,
            chull:     None,
            ancestors: PriorityQueue::new(),
            iv1:       0,
            iv2:       0
        }
    }

    pub fn is_valid(&self, dual_graph: &[DualGraphVertex]) -> bool {
        self.timestamp >= dual_graph[self.v1].timestamp &&
        self.timestamp >= dual_graph[self.v2].timestamp
    }

    pub fn compute_decimation_cost(&mut self,
                                   dual_graph:    &[DualGraphVertex],
                                   rays:          &[Ray],
                                   max_concavity: Scalar) {
        assert!(self.is_valid(dual_graph));

        // estimate the concavity.
        let v1 = &dual_graph[self.v1];
        let v2 = &dual_graph[self.v2];

        if self.chull.is_none() {
            /*
             * Compute the convex hull.
             */
            let mut vtx1 = v1.chull.as_ref().expect("Internal error: 2").mesh().coords.clone();
            let     vtx2 = &v2.chull.as_ref().expect("Internal error: 3").mesh().coords;

            vtx1.push_all(vtx2.as_slice());

            // FIXME: use a method to merge convex hulls instead of reconstructing it from scratch.
            let chull = Convex::new_with_margin(vtx1.as_slice(), na::zero());
            self.chull = Some(box chull);
        }

        let chull = self.chull.as_ref().unwrap();

        /*
         * Estimate the concavity.
         */
        let _M: Scalar = Bounded::max_value();
        let mut concavity = -self.mcost;

        let a1 = v1.ancestors.as_ref().unwrap();
        let a2 = v2.ancestors.as_ref().unwrap();

        while (self.iv1 != a1.len() || self.iv2 != a2.len()) && concavity <= max_concavity {
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
                self.ancestors.push(VertexWithConcavity::new(id, na::zero()));
                continue;
            }

            let sv   = chull.support_point(&Identity::new(), &ray.dir);
            let dist = na::dot(&sv, &ray.dir);

            if !na::approx_eq(&dist, &na::zero()) {
                let shift: Scalar = na::cast(0.1f64);
                let outside_point = ray.orig + ray.dir * (dist + shift);

                match chull.toi_with_ray(&Ray::new(outside_point, -ray.dir), true) {
                    None      => {
                        // println!("{}", 0.0f32);
                        self.ancestors.push(VertexWithConcavity::new(id, na::zero()))
                    },
                    Some(toi) => {
                        let new_concavity = dist + shift - toi;
                        // println!("{}", new_concavity);
                        self.ancestors.push(VertexWithConcavity::new(id, new_concavity));
                        if concavity < new_concavity {
                            concavity = new_concavity
                        }
                    }
                }
            }
            else {
                self.ancestors.push(VertexWithConcavity::new(id, na::zero()));
            }
        }

        self.mcost = -concavity;
        self.exact = self.iv1 == a1.len() && self.iv2 == a2.len();
    }
}

#[dim3]
impl PartialEq for DualGraphEdge {
    #[inline]
    fn eq(&self, other: &DualGraphEdge) -> bool {
        self.mcost.eq(&other.mcost)
    }
}

#[dim3]
impl Eq for DualGraphEdge {
}

#[dim3]
impl PartialOrd for DualGraphEdge {
    #[inline]
    fn partial_cmp(&self, other: &DualGraphEdge) -> Option<Ordering> {
        self.mcost.partial_cmp(&other.mcost)
    }
}

#[dim3]
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

#[dim3]
struct VertexWithConcavity {
    id:        uint,
    concavity: Scalar
}

#[dim3]
impl VertexWithConcavity {
    #[inline]
    pub fn new(id: uint, concavity: Scalar) -> VertexWithConcavity {
        VertexWithConcavity {
            id:        id,
            concavity: concavity
        }
    }
}

#[dim3]
impl PartialEq for VertexWithConcavity {
    #[inline]
    fn eq(&self, other: &VertexWithConcavity) -> bool {
        self.concavity == other.concavity && self.id == other.id
    }
}

#[dim3]
impl Eq for VertexWithConcavity {
}

#[dim3]
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

#[dim3]
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

#[dim3]
#[inline]
fn edge(a: u32, b: u32) -> Vec2<uint> {
    if a > b {
        Vec2::new(b as uint, a as uint)
    }
    else {
        Vec2::new(a as uint, b as uint)
    }
}

#[dim3]
fn compute_rays(mesh: &TriMesh<Scalar, Vec3<Scalar>>) -> (Vec<Ray>, HashMap<(u32, u32), uint>) {
    let mut rays   = Vec::new();
    let mut raymap = HashMap::new();

    {
        let coords  = mesh.coords.as_slice();
        let normals = mesh.normals.as_ref().unwrap().as_slice();

        let add_ray = |coord: u32, normal: u32| {
            let existing = raymap.find_or_insert((coord, normal), rays.len());

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

#[dim3]
fn compute_dual_graph(mesh:   &TriMesh<Scalar, Vec3<Scalar>>,
                      raymap: &HashMap<(u32, u32), uint>)
                      -> Vec<DualGraphVertex> {
    let mut rng           = IsaacRng::new_unseeded();
    let mut prim_edges    = HashMap::with_hasher(SipHasher::new_with_keys(rng.gen(), rng.gen()));
    let mut dual_vertices = Vec::from_fn(mesh.num_triangles(), |i| DualGraphVertex::new(i, mesh, raymap));

    {
        let add_triangle_edges = |i: uint, t: &Vec3<u32>| {
            let es = [ edge(t.x, t.y), edge(t.y, t.z), edge(t.z, t.x) ];

            for e in es.iter() {
                let other = prim_edges.find_or_insert(e.clone(), i);

                if *other != i {
                    // register the adjascency.
                    dual_vertices.get_mut(i).neighbors.as_mut().expect("Internal error: 13").insert(*other);
                    dual_vertices.get_mut(*other).neighbors.as_mut().expect("Internal error: 14").insert(i);
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

#[cfg(test)]
mod test {
    use std::num::Bounded;
    use std::rand;
    use nalgebra::na::Vec3;
    use procedural::{TriMesh, UnifiedIndexBuffer};
    use procedural;
    use math::Scalar;

    #[dim3]
    #[test]
    fn test_full_decimation() {
        let mesh = procedural::sphere(&0.4f32, 20, 20, true);

        let decomp = procedural::hacd(&mesh, Bounded::max_value(), 1);

        assert!(decomp.len() == 1, format!("Expected 1 hull instead of {}.", decomp.len()));
        // FIXME: test that it matches the mesh's convex hull.
    }
}
