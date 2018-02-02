use std::marker::PhantomData;
use std::collections::BinaryHeap;
use std::cmp::Ordering;
use approx::ApproxEq;

use alga::general::{Id, Real};
use na::{self, Unit};

use utils;
use shape::{AnnotatedMinkowskiSum, AnnotatedPoint, Reflection, SupportMap};
use query::algorithms::gjk;
use query::algorithms::simplex::Simplex;
use math::Point;

#[derive(Copy, Clone, PartialEq)]
struct FaceId<N: Real> {
    id: usize,
    neg_dist: N,
}

impl<N: Real> FaceId<N> {
    fn new(id: usize, neg_dist: N) -> Self {
        assert!(neg_dist <= gjk::eps_tol());
        FaceId { id, neg_dist }
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

#[derive(Clone)]
struct Face<P: Point> {
    pts: [usize; 2],
    normal: Unit<P::Vector>,
    proj: P,
    deleted: bool,
    _marker: PhantomData<P>,
}

impl<P: Point> Face<P> {
    pub fn new(vertices: &[P], pts: [usize; 2]) -> (Self, bool) {
        if let Some(proj) = project_origin(&vertices[pts[0]], &vertices[pts[1]]) {
            (Self::new_with_proj(vertices, proj, pts), true)
        } else {
            (Self::new_with_proj(vertices, P::origin(), pts), false)
        }
    }

    pub fn new_with_proj(vertices: &[P], proj: P, pts: [usize; 2]) -> Self {
        let normal;
        let deleted;

        if let Some(n) = P::ccw_face_normal(&[&vertices[pts[0]], &vertices[pts[1]]]) {
            normal = n;
            deleted = false;
        } else {
            normal = Unit::new_unchecked(na::zero());
            deleted = true;
        }

        let _marker = PhantomData;

        Face {
            pts,
            normal,
            proj,
            deleted,
            _marker,
        }
    }
}

pub struct EPA2<P: Point> {
    vertices: Vec<P>,
    faces: Vec<Face<P>>,
    heap: BinaryHeap<FaceId<P::Real>>,
}

impl<P: Point> EPA2<P> {
    pub fn new() -> Self {
        EPA2 {
            vertices: Vec::new(),
            faces: Vec::new(),
            heap: BinaryHeap::new(),
        }
    }

    fn reset(&mut self) {
        self.vertices.clear();
        self.faces.clear();
        self.heap.clear();
    }

    /// Projects the origin on a shape unsing the EPA algorithm.
    ///
    /// The origin is assumed to be located inside of the shape.
    pub fn project_origin<M, S, G: ?Sized>(&mut self, m: &M, shape: &G, simplex: &S) -> P
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
            return P::origin();
        } else if simplex.dimension() == 2 {
            let dp1 = self.vertices[1] - self.vertices[0];
            let dp2 = self.vertices[2] - self.vertices[0];

            if utils::perp2(&dp1, &dp2) < na::zero() {
                self.vertices.swap(1, 2)
            }

            let pts1 = [0, 1];
            let pts2 = [1, 2];
            let pts3 = [2, 0];

            let (face1, proj_is_inside1) = Face::new(&self.vertices, pts1);
            let (face2, proj_is_inside2) = Face::new(&self.vertices, pts2);
            let (face3, proj_is_inside3) = Face::new(&self.vertices, pts3);

            self.faces.push(face1);
            self.faces.push(face2);
            self.faces.push(face3);

            if proj_is_inside1 {
                let dist1 = na::dot(
                    self.faces[0].normal.as_ref(),
                    &self.vertices[0].coordinates(),
                );
                self.heap.push(FaceId::new(0, -dist1));
            }

            if proj_is_inside2 {
                let dist2 = na::dot(
                    self.faces[1].normal.as_ref(),
                    &self.vertices[1].coordinates(),
                );
                self.heap.push(FaceId::new(1, -dist2));
            }

            if proj_is_inside3 {
                let dist3 = na::dot(
                    self.faces[2].normal.as_ref(),
                    &self.vertices[2].coordinates(),
                );
                self.heap.push(FaceId::new(2, -dist3));
            }
        } else {
            let pts1 = [0, 1];
            let pts2 = [1, 0];

            self.faces
                .push(Face::new_with_proj(&self.vertices, P::origin(), pts1));
            self.faces
                .push(Face::new_with_proj(&self.vertices, P::origin(), pts2));

            let dist1 = na::dot(
                self.faces[0].normal.as_ref(),
                &self.vertices[0].coordinates(),
            );
            let dist2 = na::dot(
                self.faces[1].normal.as_ref(),
                &self.vertices[1].coordinates(),
            );

            self.heap.push(FaceId::new(0, dist1));
            self.heap.push(FaceId::new(1, dist2));
        }

        let mut niter = 0;
        let mut last_face_id = *self.heap.peek().unwrap();

        /*
         * Run the expansion.
         */
        while let Some(face_id) = self.heap.pop() {
            // Create new faces.
            let f1;
            let f2;

            let supp_pt_dist;
            {
                let face = &self.faces[face_id.id];
                let support_point = shape.support_point(m, &face.normal);
                let support_point_id = self.vertices.len();
                self.vertices.push(support_point);

                let pts1 = [face.pts[0], support_point_id];
                let pts2 = [support_point_id, face.pts[1]];

                f1 = Face::new(&self.vertices, pts1);
                f2 = Face::new(&self.vertices, pts2);

                supp_pt_dist = na::dot(&support_point.coordinates(), &face.normal);
            }

            let id1 = self.faces.len() + 0;
            let id2 = self.faces.len() + 1;

            if f1.1 {
                let dist1 = na::dot(f1.0.normal.as_ref(), &f1.0.proj.coordinates());
                self.heap.push(FaceId::new(id1, -dist1));
            }
            if f2.1 {
                let dist2 = na::dot(f2.0.normal.as_ref(), &f2.0.proj.coordinates());
                self.heap.push(FaceId::new(id2, -dist2));
            }

            self.faces.push(f1.0);
            self.faces.push(f2.0);

            let curr_dist = -face_id.neg_dist;

            if (curr_dist - supp_pt_dist).abs() < _eps_tol {
                return self.faces[face_id.id].proj;
            }

            last_face_id = face_id;
            niter += 1;
            if niter > 10000 {
                println!("Internal error: EPA did not converge.");
                break;
            }
        }

        return self.faces[last_face_id.id].proj;
    }
}

pub fn closest_points<P, M, S, G1: ?Sized, G2: ?Sized>(
    epa: &mut EPA2<AnnotatedPoint<P>>,
    m1: &M,
    g1: &G1,
    m2: &M,
    g2: &G2,
    simplex: &S,
) -> (P, P)
where
    P: Point,
    S: Simplex<AnnotatedPoint<P>>,
    G1: SupportMap<P, M>,
    G2: SupportMap<P, M>,
{
    let reflect2 = Reflection::new(g2);
    let cso = AnnotatedMinkowskiSum::new(m1, g1, m2, &reflect2);

    let p = epa.project_origin(&Id::new(), &cso, simplex);
    (*p.orig1(), -*p.orig2())
}

fn project_origin<P: Point>(a: &P, b: &P) -> Option<P> {
    let ab = *b - *a;
    let ap = -a.coordinates();
    let ab_ap = na::dot(&ab, &ap);
    let sqnab = na::norm_squared(&ab);

    if sqnab == na::zero() {
        return None;
    }

    let position_on_segment;

    let _eps = P::Real::default_epsilon();

    if ab_ap < -_eps || ab_ap > sqnab + P::Real::default_epsilon() {
        // Voronoï region of vertex 'a' or 'b'.
        None
    } else {
        // Voronoï region of the segment interior.
        position_on_segment = ab_ap / sqnab;

        let mut res = *a;
        let _1 = na::one::<P::Real>();
        res.axpy(position_on_segment, b, (_1 - position_on_segment));

        Some(res)
    }
}
