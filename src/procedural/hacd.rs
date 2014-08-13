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
pub fn hacd(mesh: &TriMesh<Scalar, Vec3<Scalar>>, error: Scalar, min_components: uint, max_vertex_per_hull: uint) -> Vec<Convex> {
    assert!(mesh.normals.is_some(), "Vertex normals are required to compute the convex decomposition.");

    let mut edges      = PriorityQueue::new();
    let mut dual_graph = compute_dual_graph(mesh);

    /*
     * Initialize the priority queue.
     */
    for (i, v) in dual_graph.iter().enumerate() {
        for n in v.neighbors.as_ref().expect("Internal error: 1").iter() {
            if i < *n {
                let edge = DualGraphEdge::new(0, i, *n, dual_graph.as_slice());

                edges.push(edge);
            }
        }
    }

    println!("Number of edges: {}", edges.len());

    /*
     * Decimation.
     */
    let mut curr_time = 0;
    let mut useful_optim = 0u;
    let mut exact_computations = 0u;
    let mut rejected_exact = 0u;
    let mut pushedback_exact = 0u;
    let mut accepted_exact = 0u;

    while dual_graph.len() - curr_time > min_components {
        let to_add = match edges.pop() {
            None      => break,
            Some(mut top) => {
                if top.timestamp < dual_graph[top.v1].timestamp ||
                   top.timestamp < dual_graph[top.v2].timestamp {
                    if !top.exact {
                        useful_optim = useful_optim + 1;
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
                                if e.timestamp < dual_graph[e.v1].timestamp || e.timestamp < dual_graph[e.v2].timestamp {
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

                    let mut vtx1 = dual_graph[top.v1].chull.as_ref().expect("Internal error: 2").mesh().coords.clone();
                    let     vtx2 = &dual_graph[top.v2].chull.as_ref().expect("Internal error: 3").mesh().coords;

                    vtx1.push_all(vtx2.as_slice());

                    // FIXME: use a method to merge convex hulls instead of reconstructing it from scratch.
                    let chull = Convex::new_with_margin(vtx1.as_slice(), na::zero());


                    // FIXME: move this as a method of the edge?
                    top.mcost = -DualGraphVertex::decimation_cost(&dual_graph[top.v1],
                                                                  &dual_graph[top.v2],
                                                                  mesh,
                                                                  &chull,
                                                                  &error);
                    top.chull = Some(box chull);
                    top.exact = true;

                    exact_computations = exact_computations + 1;

                    if top.mcost < top_cost {
                        // we are not the greatest cost any more.
                        edges.push(top);
                        pushedback_exact = pushedback_exact + 1;
                        continue;
                    }
                }

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
    ancestors: Option<HashSet<uint, SipHasher>>, // faces from the original surface.
    timestamp: uint,
    chull:     Option<Convex>,
    mcost:     Scalar
}

#[dim3]
impl DualGraphVertex {
    pub fn new(ancestor: uint, mesh: &TriMesh<Scalar, Vec3<Scalar>>) -> DualGraphVertex {
        let idx =
            match mesh.indices {
                UnifiedIndexBuffer(ref idx) => idx[ancestor].clone(),
                SplitIndexBuffer(ref idx) => {
                    let t = idx[ancestor];
                    Vec3::new(t.x.x, t.y.x, t.z.x)
                }
            };

        let triangle = vec!(mesh.coords[idx.x as uint],
                            mesh.coords[idx.y as uint],
                            mesh.coords[idx.z as uint]);

        let chull = unsafe {;
            Convex::new_with_convex_mesh(TriMesh::new(triangle, None, None, None), na::zero())
        };

        let mut rng       = IsaacRng::new_unseeded();
        let mut ancestors = HashSet::with_hasher(SipHasher::new_with_keys(rng.gen(), rng.gen()));
        ancestors.insert(idx.x as uint);
        ancestors.insert(idx.y as uint);
        ancestors.insert(idx.z as uint);

        DualGraphVertex {
            neighbors: Some(HashSet::with_hasher(SipHasher::new_with_keys(rng.gen(), rng.gen()))),
            ancestors: Some(ancestors),
            timestamp: 0,
            chull:     Some(chull),
            mcost:     na::zero()
        }
    }

    pub fn hecol(edge: DualGraphEdge, graph: &mut [DualGraphVertex]) {
        let valid = edge.v1;
        let other = edge.v2;

        graph[valid].chull = None;
        graph[other].chull = None;

        let other_neighbors = graph[other].neighbors.take_unwrap();
        let other_ancestors = graph[other].ancestors.take_unwrap();

        for neighbor in other_neighbors.iter() {
            if *neighbor != valid {
                let ga = &mut graph[*neighbor];

                // replace `other` by `valid`.
                ga.neighbors.as_mut().expect("Internal error: 6").remove(&other);
                ga.neighbors.as_mut().expect("Internal error: 7").insert(valid);
            }
        }

        let gvalid = &mut graph[valid];
        gvalid.chull = Some(*edge.chull.expect("Internal error: 8"));
        gvalid.neighbors.as_mut().expect("Internal error: 9").extend(other_neighbors.move_iter());
        gvalid.neighbors.as_mut().expect("Internal error: 10").remove(&other);
        gvalid.neighbors.as_mut().expect("Internal error: 11").remove(&valid);
        gvalid.ancestors.as_mut().unwrap().extend(other_ancestors.move_iter());
        gvalid.mcost = edge.mcost;
    }

    pub fn decimation_cost(v1:    &DualGraphVertex,
                           v2:    &DualGraphVertex,
                           mesh:  &TriMesh<Scalar, Vec3<Scalar>>,
                           hull:  &Convex,
                           _:     &Scalar) -> Scalar {
        // estimate the concavity.
        let mut concavity  = na::zero::<Scalar>();

        let normals = mesh.normals.as_ref().expect("Internal error: 12").as_slice();
        let coords  = mesh.coords.as_slice();

        for id in v1.ancestors.as_ref().unwrap().iter().chain(v2.ancestors.as_ref().unwrap().iter()) {
            let mut n = normals[*id].clone();

            if n.normalize().is_zero() {
                continue;
            }

            let v = &coords[*id];

            let sv   = hull.support_point(&Identity::new(), &n);
            let dist = na::dot(&sv, &n);

            if !na::approx_eq(&dist, &na::zero()) {
                let shift: Scalar = na::cast(0.1f64);
                let outside_point = *v + n * (dist + shift);

                match hull.toi_with_ray(&Ray::new(outside_point, -n), true) {
                    None      => { },
                    Some(toi) => {
                        let new_concavity = dist + shift - toi;
                        if concavity < new_concavity {
                            concavity = new_concavity
                        }
                    }
                }
            }
        }

        // println!("Cost: {}", concavity);

        concavity
    }
}

#[dim3]
struct DualGraphEdge {
    v1:        uint,
    v2:        uint,
    mcost:     Scalar,
    exact:     bool,
    timestamp: uint,
    chull:     Option<Box<Convex>>
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
            mcost:     dual_graph[v1].mcost.max(dual_graph[v2].mcost),
            exact:     false,
            timestamp: timestamp,
            chull:     None
        }
    }
}

#[dim3]
impl PartialEq for DualGraphEdge {
    fn eq(&self, other: &DualGraphEdge) -> bool {
        self.mcost.eq(&other.mcost)
    }
}

#[dim3]
impl Eq for DualGraphEdge {
}

#[dim3]
impl PartialOrd for DualGraphEdge {
    fn partial_cmp(&self, other: &DualGraphEdge) -> Option<Ordering> {
        self.mcost.partial_cmp(&other.mcost)
    }
}

#[dim3]
impl Ord for DualGraphEdge {
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
fn edge(a: u32, b: u32) -> Vec2<uint> {
    if a > b {
        Vec2::new(b as uint, a as uint)
    }
    else {
        Vec2::new(a as uint, b as uint)
    }
}

#[dim3]
fn compute_dual_graph(mesh: &TriMesh<Scalar, Vec3<Scalar>>) -> Vec<DualGraphVertex> {
    let mut rng           = IsaacRng::new_unseeded();
    let mut prim_edges    = HashMap::with_hasher(SipHasher::new_with_keys(rng.gen(), rng.gen()));
    let mut dual_vertices = Vec::from_fn(mesh.num_triangles(), |i| DualGraphVertex::new(i, mesh));

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
