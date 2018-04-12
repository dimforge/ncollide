//! Three-dimensional penetration depth queries using the Expanding Polytope Algorithm.

use std::marker::PhantomData;
use std::collections::BinaryHeap;
use std::cmp::Ordering;
use num::Bounded;
use approx::ApproxEq;

use alga::general::{Id, Real};
use alga::linear::FiniteDimInnerSpace;
use na::{self, Unit};

use utils;
use shape::{AnnotatedMinkowskiSum, AnnotatedPoint, Reflection, SupportMap, Triangle};
use query::PointQueryWithLocation;
use query::algorithms::gjk;
use query::algorithms::simplex::Simplex;
use math::Point;

#[derive(Copy, Clone, PartialEq)]
struct FaceId<N: Real> {
    id: usize,
    neg_dist: N,
}

impl<N: Real> FaceId<N> {
    fn new(id: usize, neg_dist: N) -> Option<Self> {
        if neg_dist > gjk::eps_tol() {
            println!(
                "EPA: the origin was outside of the CSO: {} > tolerence ({})",
                neg_dist,
                gjk::eps_tol::<N>()
            );
            None
        } else {
            Some(FaceId { id, neg_dist })
        }
    }
}

impl<N: Real> Eq for FaceId<N> {}

impl<N: Real> PartialOrd for FaceId<N> {
    #[inline]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.neg_dist.partial_cmp(&other.neg_dist)
    }
}

impl<N: Real> Ord for FaceId<N> {
    #[inline]
    fn cmp(&self, other: &Self) -> Ordering {
        if self.neg_dist < other.neg_dist {
            Ordering::Less
        } else if self.neg_dist > other.neg_dist {
            Ordering::Greater
        } else {
            Ordering::Equal
        }
    }
}

#[derive(Clone, Debug)]
struct Face<P: Point> {
    pts: [usize; 3],
    adj: [usize; 3],
    normal: Unit<P::Vector>,
    proj: P,
    deleted: bool,
    _marker: PhantomData<P>,
}

impl<P: Point> Face<P> {
    pub fn new_with_proj(vertices: &[P], proj: P, pts: [usize; 3], adj: [usize; 3]) -> Self {
        let normal;
        let deleted;

        if let Some(n) =
            P::ccw_face_normal(&[&vertices[pts[0]], &vertices[pts[1]], &vertices[pts[2]]])
        {
            normal = n;
            deleted = false;
        } else {
            normal = Unit::new_unchecked(na::zero());
            deleted = true;
        }

        let _marker = PhantomData;

        Face {
            pts,
            proj,
            adj,
            normal,
            deleted,
            _marker,
        }
    }

    pub fn new(vertices: &[P], pts: [usize; 3], adj: [usize; 3]) -> (Self, bool) {
        let tri = Triangle::new(vertices[pts[0]], vertices[pts[1]], vertices[pts[2]]);
        let (proj, loc) = tri.project_point_with_location(&Id::new(), &P::origin(), true);

        (
            Self::new_with_proj(vertices, proj.point, pts, adj),
            loc.is_on_face(),
        )
    }

    pub fn contains_point(&self, id: usize) -> bool {
        self.pts[0] == id || self.pts[1] == id || self.pts[2] == id
    }

    pub fn next_ccw_pt_id(&self, id: usize) -> usize {
        if self.pts[0] == id {
            1
        } else if self.pts[1] == id {
            2
        } else {
            assert!(self.pts[2] == id);
            0
        }
    }

    pub fn can_be_seen_by(&self, vertices: &[P], point: usize, opp_pt_id: usize) -> bool {
        let p0 = &vertices[self.pts[opp_pt_id]];
        let p1 = &vertices[self.pts[(opp_pt_id + 1) % 3]];
        let p2 = &vertices[self.pts[(opp_pt_id + 2) % 3]];
        let pt = &vertices[point];
        na::dot(&(*pt - *p0), &self.normal) >= -gjk::eps_tol::<P::Real>()
            || utils::is_affinely_dependent_triangle3(p1, p2, pt)
    }
}

struct SilhouetteEdge {
    face_id: usize,
    opp_pt_id: usize,
}

impl SilhouetteEdge {
    pub fn new(face_id: usize, opp_pt_id: usize) -> Self {
        SilhouetteEdge { face_id, opp_pt_id }
    }
}

/// The Expanding Polytope Algorithm in 3D.
pub struct EPA3<P: Point> {
    vertices: Vec<P>,
    faces: Vec<Face<P>>,
    silhouette: Vec<SilhouetteEdge>,
    heap: BinaryHeap<FaceId<P::Real>>,
}

impl<P: Point> EPA3<P> {
    /// Creates a new instance of the 3D Expanding Polytope Algorithm.
    pub fn new() -> Self {
        EPA3 {
            vertices: Vec::new(),
            faces: Vec::new(),
            silhouette: Vec::new(),
            heap: BinaryHeap::new(),
        }
    }

    fn reset(&mut self) {
        self.vertices.clear();
        self.faces.clear();
        self.heap.clear();
        self.silhouette.clear();
    }

    /// Projects the origin on a shape unsing the EPA algorithm.
    ///
    /// The origin is assumed to be located inside of the shape.
    /// Returns `None` if the EPA fails to converge or if `g1` and `g2` are not penetrating.
    pub fn project_origin<M, S, G: ?Sized>(
        &mut self,
        m: &M,
        shape: &G,
        simplex: &S,
    ) -> Option<(P, Unit<P::Vector>)>
    where
        S: Simplex<P>,
        G: SupportMap<P, M>,
    {
        let _eps = P::Real::default_epsilon();
        let _eps_tol = _eps * na::convert(100.0f64);

        self.reset();

        /*
         * Initialization.
         */
        for i in 0..simplex.dimension() + 1 {
            self.vertices.push(simplex.point(i));
        }

        if simplex.dimension() == 0 {
            let mut n: P::Vector = na::zero();
            n[1] = na::one();
            return Some((P::origin(), Unit::new_unchecked(n)));
        } else if simplex.dimension() == 3 {
            let dp1 = self.vertices[1] - self.vertices[0];
            let dp2 = self.vertices[2] - self.vertices[0];
            let dp3 = self.vertices[3] - self.vertices[0];

            if na::dot(&utils::cross3(&dp1, &dp2), &dp3) > na::zero() {
                self.vertices.swap(1, 2)
            }

            let pts1 = [0, 1, 2];
            let pts2 = [1, 3, 2];
            let pts3 = [0, 2, 3];
            let pts4 = [0, 3, 1];

            let adj1 = [3, 1, 2];
            let adj2 = [3, 2, 0];
            let adj3 = [0, 1, 3];
            let adj4 = [2, 1, 0];

            let (face1, proj_inside1) = Face::new(&self.vertices, pts1, adj1);
            let (face2, proj_inside2) = Face::new(&self.vertices, pts2, adj2);
            let (face3, proj_inside3) = Face::new(&self.vertices, pts3, adj3);
            let (face4, proj_inside4) = Face::new(&self.vertices, pts4, adj4);

            self.faces.push(face1);
            self.faces.push(face2);
            self.faces.push(face3);
            self.faces.push(face4);

            if proj_inside1 {
                let dist1 = na::dot(
                    self.faces[0].normal.as_ref(),
                    &self.vertices[0].coordinates(),
                );
                self.heap.push(FaceId::new(0, -dist1)?);
            }

            if proj_inside2 {
                let dist2 = na::dot(
                    self.faces[1].normal.as_ref(),
                    &self.vertices[1].coordinates(),
                );
                self.heap.push(FaceId::new(1, -dist2)?);
            }

            if proj_inside3 {
                let dist3 = na::dot(
                    self.faces[2].normal.as_ref(),
                    &self.vertices[2].coordinates(),
                );
                self.heap.push(FaceId::new(2, -dist3)?);
            }

            if proj_inside4 {
                let dist4 = na::dot(
                    self.faces[3].normal.as_ref(),
                    &self.vertices[3].coordinates(),
                );
                self.heap.push(FaceId::new(3, -dist4)?);
            }
        } else {
            if simplex.dimension() == 1 {
                let dpt = self.vertices[1] - self.vertices[0];

                P::Vector::orthonormal_subspace_basis(&[dpt], |dir| {
                    self.vertices.push(shape.support_point(m, dir));
                    false
                });
            }

            let pts1 = [0, 1, 2];
            let pts2 = [0, 2, 1];

            let adj1 = [1, 1, 1];
            let adj2 = [0, 0, 0];

            let (face1, _) = Face::new(&self.vertices, pts1, adj1);
            let (face2, _) = Face::new(&self.vertices, pts2, adj2);
            self.faces.push(face1);
            self.faces.push(face2);

            self.heap.push(FaceId::new(0, na::zero())?);
            self.heap.push(FaceId::new(1, na::zero())?);
        }

        let mut niter = 0;
        let mut max_dist = P::Real::max_value();
        let mut best_face_id = *self.heap.peek().unwrap();

        /*
         * Run the expansion.
         */
        while let Some(face_id) = self.heap.pop() {
            // Create new faces.
            let face = self.faces[face_id.id].clone();

            if face.deleted {
                continue;
            }

            let support_point = shape.support_point(m, &face.normal);
            let support_point_id = self.vertices.len();
            self.vertices.push(support_point);

            let candidate_max_dist = na::dot(&support_point.coordinates(), &face.normal);

            if candidate_max_dist < max_dist {
                best_face_id = face_id;
                max_dist = candidate_max_dist;
            }

            let curr_dist = -face_id.neg_dist;

            if max_dist - curr_dist < _eps_tol {
                let best_face = &self.faces[best_face_id.id];
                return Some((best_face.proj, best_face.normal));
            }

            self.faces[face_id.id].deleted = true;

            let adj_opp_pt_id1 = self.faces[face.adj[0]].next_ccw_pt_id(face.pts[0]);
            let adj_opp_pt_id2 = self.faces[face.adj[1]].next_ccw_pt_id(face.pts[1]);
            let adj_opp_pt_id3 = self.faces[face.adj[2]].next_ccw_pt_id(face.pts[2]);

            self.compute_silhouette(support_point_id, face.adj[0], adj_opp_pt_id1);
            self.compute_silhouette(support_point_id, face.adj[1], adj_opp_pt_id2);
            self.compute_silhouette(support_point_id, face.adj[2], adj_opp_pt_id3);

            let first_new_face_id = self.faces.len();
            for edge in &self.silhouette {
                if !self.faces[edge.face_id].deleted {
                    let new_face_id = self.faces.len();
                    let new_face;

                    // FIXME: NLL
                    {
                        let face_adj = &mut self.faces[edge.face_id];
                        let pt_id1 = face_adj.pts[(edge.opp_pt_id + 2) % 3];
                        let pt_id2 = face_adj.pts[(edge.opp_pt_id + 1) % 3];

                        let pts = [pt_id1, pt_id2, support_point_id];
                        let adj = [edge.face_id, new_face_id + 1, new_face_id - 1];
                        new_face = Face::new(&self.vertices, pts, adj);

                        face_adj.adj[(edge.opp_pt_id + 1) % 3] = new_face_id;
                    }

                    self.faces.push(new_face.0);

                    if new_face.1 {
                        let pt = self.vertices[self.faces[new_face_id].pts[0]].coordinates();
                        let dist = na::dot(self.faces[new_face_id].normal.as_ref(), &pt);
                        if dist < curr_dist {
                            // FIXME: if we reach this point, there were issues due to
                            // numerical errors.
                            return Some((face.proj, face.normal));
                        }

                        self.heap.push(FaceId::new(new_face_id, -dist)?);
                    }
                }
            }

            self.faces[first_new_face_id].adj[2] = self.faces.len() - 1;
            self.faces.last_mut().unwrap().adj[1] = first_new_face_id;

            self.silhouette.clear();
            // self.check_topology(); // NOTE: for debugging only.

            niter += 1;
            if niter > 10000 {
                println!("EPA did not converge after 1000 iterations… stopping the iterations.");
                return None;
            }
        }

        let best_face = &self.faces[best_face_id.id];
        return Some((best_face.proj, best_face.normal));
    }

    fn compute_silhouette(&mut self, point: usize, id: usize, opp_pt_id: usize) {
        if !self.faces[id].deleted {
            if !self.faces[id].can_be_seen_by(&self.vertices, point, opp_pt_id) {
                self.silhouette.push(SilhouetteEdge::new(id, opp_pt_id));
            } else {
                self.faces[id].deleted = true;

                let adj_pt_id1 = (opp_pt_id + 2) % 3;
                let adj_pt_id2 = opp_pt_id;

                let adj1 = self.faces[id].adj[adj_pt_id1];
                let adj2 = self.faces[id].adj[adj_pt_id2];

                let adj_opp_pt_id1 =
                    self.faces[adj1].next_ccw_pt_id(self.faces[id].pts[adj_pt_id1]);
                let adj_opp_pt_id2 =
                    self.faces[adj2].next_ccw_pt_id(self.faces[id].pts[adj_pt_id2]);

                self.compute_silhouette(point, adj1, adj_opp_pt_id1);
                self.compute_silhouette(point, adj2, adj_opp_pt_id2);
            }
        }
    }

    #[allow(dead_code)]
    fn print_silhouette(&self) {
        print!("Silhouette points: ");
        for i in 0..self.silhouette.len() {
            let edge = &self.silhouette[i];
            let face = &self.faces[edge.face_id];

            if !face.deleted {
                print!(
                    "({}, {}) ",
                    face.pts[(edge.opp_pt_id + 2) % 3],
                    face.pts[(edge.opp_pt_id + 1) % 3]
                );
            }
        }
        println!("");
    }

    #[allow(dead_code)]
    fn check_topology(&self) {
        for i in 0..self.faces.len() {
            let face = &self.faces[i];
            if face.deleted {
                continue;
            }

            println!("checking {}-th face.", i);
            let adj1 = &self.faces[face.adj[0]];
            let adj2 = &self.faces[face.adj[1]];
            let adj3 = &self.faces[face.adj[2]];

            assert!(!adj1.deleted);
            assert!(!adj2.deleted);
            assert!(!adj3.deleted);

            assert!(face.pts[0] != face.pts[1]);
            assert!(face.pts[0] != face.pts[2]);
            assert!(face.pts[1] != face.pts[2]);

            assert!(adj1.contains_point(face.pts[0]));
            assert!(adj1.contains_point(face.pts[1]));

            assert!(adj2.contains_point(face.pts[1]));
            assert!(adj2.contains_point(face.pts[2]));

            assert!(adj3.contains_point(face.pts[2]));
            assert!(adj3.contains_point(face.pts[0]));

            let opp_pt_id1 = adj1.next_ccw_pt_id(face.pts[0]);
            let opp_pt_id2 = adj2.next_ccw_pt_id(face.pts[1]);
            let opp_pt_id3 = adj3.next_ccw_pt_id(face.pts[2]);

            assert!(!face.contains_point(adj1.pts[opp_pt_id1]));
            assert!(!face.contains_point(adj2.pts[opp_pt_id2]));
            assert!(!face.contains_point(adj3.pts[opp_pt_id3]));

            assert!(adj1.adj[(opp_pt_id1 + 1) % 3] == i);
            assert!(adj2.adj[(opp_pt_id2 + 1) % 3] == i);
            assert!(adj3.adj[(opp_pt_id3 + 1) % 3] == i);
        }
    }
}

/// Computes the pair of closest points at the extremities of the minimal translational vector between `g1` and `g2`.
pub fn closest_points<P, M, S, G1: ?Sized, G2: ?Sized>(
    epa: &mut EPA3<AnnotatedPoint<P>>,
    m1: &M,
    g1: &G1,
    m2: &M,
    g2: &G2,
    simplex: &S,
) -> Option<(P, P, Unit<P::Vector>)>
where
    P: Point,
    S: Simplex<AnnotatedPoint<P>>,
    G1: SupportMap<P, M>,
    G2: SupportMap<P, M>,
{
    let reflect2 = Reflection::new(g2);
    let cso = AnnotatedMinkowskiSum::new(m1, g1, m2, &reflect2);

    let (p, n) = epa.project_origin(&Id::new(), &cso, simplex)?;
    Some((*p.orig1(), -*p.orig2(), n))
}
