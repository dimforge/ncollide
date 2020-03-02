//! Two-dimensional penetration depth queries using the Expanding Polytope Algorithm.

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use alga::general::RealField;
use na::{self, Unit};

use crate::math::{Isometry, Point, Vector};
use crate::query::algorithms::{
    gjk, special_support_maps::ConstantOrigin, CSOPoint, VoronoiSimplex,
};
use crate::shape::SupportMap;
use crate::utils;

#[derive(Copy, Clone, PartialEq)]
struct FaceId<N: RealField> {
    id: usize,
    neg_dist: N,
}

impl<N: RealField> FaceId<N> {
    fn new(id: usize, neg_dist: N) -> Option<Self> {
        if neg_dist > gjk::eps_tol() {
            //            println!(
            //                "EPA: the origin was outside of the CSO: {} > tolerence ({})",
            //                neg_dist,
            //                gjk::eps_tol::<N>()
            //            );
            None
        } else {
            Some(FaceId { id, neg_dist })
        }
    }
}

impl<N: RealField> Eq for FaceId<N> {}

impl<N: RealField> PartialOrd for FaceId<N> {
    #[inline]
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        self.neg_dist.partial_cmp(&other.neg_dist)
    }
}

impl<N: RealField> Ord for FaceId<N> {
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
struct Face<N: RealField> {
    pts: [usize; 2],
    normal: Unit<Vector<N>>,
    proj: Point<N>,
    bcoords: [N; 2],
    deleted: bool,
}

impl<N: RealField> Face<N> {
    pub fn new(vertices: &[CSOPoint<N>], pts: [usize; 2]) -> (Self, bool) {
        if let Some((proj, bcoords)) =
            project_origin(&vertices[pts[0]].point, &vertices[pts[1]].point)
        {
            (Self::new_with_proj(vertices, proj, bcoords, pts), true)
        } else {
            (
                Self::new_with_proj(vertices, Point::origin(), [N::zero(); 2], pts),
                false,
            )
        }
    }

    pub fn new_with_proj(
        vertices: &[CSOPoint<N>],
        proj: Point<N>,
        bcoords: [N; 2],
        pts: [usize; 2],
    ) -> Self {
        let normal;
        let deleted;

        if let Some(n) = utils::ccw_face_normal([&vertices[pts[0]].point, &vertices[pts[1]].point])
        {
            normal = n;
            deleted = false;
        } else {
            normal = Unit::new_unchecked(na::zero());
            deleted = true;
        }

        Face {
            pts,
            normal,
            proj,
            bcoords,
            deleted,
        }
    }

    pub fn closest_points(&self, vertices: &[CSOPoint<N>]) -> (Point<N>, Point<N>) {
        (
            vertices[self.pts[0]].orig1 * self.bcoords[0]
                + vertices[self.pts[1]].orig1.coords * self.bcoords[1],
            vertices[self.pts[0]].orig2 * self.bcoords[0]
                + vertices[self.pts[1]].orig2.coords * self.bcoords[1],
        )
    }
}

/// The Expanding Polytope Algorithm in 2D.
pub struct EPA<N: RealField> {
    vertices: Vec<CSOPoint<N>>,
    faces: Vec<Face<N>>,
    heap: BinaryHeap<FaceId<N>>,
}

impl<N: RealField> EPA<N> {
    /// Creates a new instance of the 2D Expanding Polytope Algorithm.
    pub fn new() -> Self {
        EPA {
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

    /// Projects the origin on boundary the given shape.
    ///
    /// The origin is assumed to be inside of the shape. If it is outside, use
    /// the GJK algorithm instead.
    /// Return `None` if the origin is not inside of the shape or if
    /// the EPA algorithm failed to compute the projection.
    pub fn project_origin<G: ?Sized>(
        &mut self,
        m: &Isometry<N>,
        g: &G,
        simplex: &VoronoiSimplex<N>,
    ) -> Option<Point<N>>
    where
        G: SupportMap<N>,
    {
        self.closest_points(m, g, &Isometry::identity(), &ConstantOrigin, simplex)
            .map(|(p, _, _)| p)
    }

    /// Projects the origin on a shape unsing the EPA algorithm.
    ///
    /// The origin is assumed to be located inside of the shape.
    /// Returns `None` if the EPA fails to converge or if `g1` and `g2` are not penetrating.
    pub fn closest_points<G1: ?Sized, G2: ?Sized>(
        &mut self,
        m1: &Isometry<N>,
        g1: &G1,
        m2: &Isometry<N>,
        g2: &G2,
        simplex: &VoronoiSimplex<N>,
    ) -> Option<(Point<N>, Point<N>, Unit<Vector<N>>)>
    where
        G1: SupportMap<N>,
        G2: SupportMap<N>,
    {
        let _eps = N::default_epsilon();
        let _eps_tol = _eps * na::convert(100.0f64);

        self.reset();

        /*
         * Initialization.
         */
        for i in 0..simplex.dimension() + 1 {
            self.vertices.push(*simplex.point(i));
        }

        if simplex.dimension() == 0 {
            let mut n: Vector<N> = na::zero();
            n[1] = na::one();
            return Some((Point::origin(), Point::origin(), Unit::new_unchecked(n)));
        } else if simplex.dimension() == 2 {
            let dp1 = self.vertices[1] - self.vertices[0];
            let dp2 = self.vertices[2] - self.vertices[0];

            if dp1.perp(&dp2) < na::zero() {
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
                let dist1 = self.faces[0].normal.dot(&self.vertices[0].point.coords);
                self.heap.push(FaceId::new(0, -dist1)?);
            }

            if proj_is_inside2 {
                let dist2 = self.faces[1].normal.dot(&self.vertices[1].point.coords);
                self.heap.push(FaceId::new(1, -dist2)?);
            }

            if proj_is_inside3 {
                let dist3 = self.faces[2].normal.dot(&self.vertices[2].point.coords);
                self.heap.push(FaceId::new(2, -dist3)?);
            }
        } else {
            let pts1 = [0, 1];
            let pts2 = [1, 0];

            self.faces.push(Face::new_with_proj(
                &self.vertices,
                Point::origin(),
                [N::one(), N::zero()],
                pts1,
            ));
            self.faces.push(Face::new_with_proj(
                &self.vertices,
                Point::origin(),
                [N::one(), N::zero()],
                pts2,
            ));

            let dist1 = self.faces[0].normal.dot(&self.vertices[0].point.coords);
            let dist2 = self.faces[1].normal.dot(&self.vertices[1].point.coords);

            self.heap.push(FaceId::new(0, dist1)?);
            self.heap.push(FaceId::new(1, dist2)?);
        }

        let mut niter = 0;
        let mut max_dist = N::max_value();
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

            let cso_point = CSOPoint::from_shapes(m1, g1, m2, g2, &face.normal);
            let support_point_id = self.vertices.len();
            self.vertices.push(cso_point);

            let candidate_max_dist = cso_point.point.coords.dot(&face.normal);

            if candidate_max_dist < max_dist {
                best_face_id = face_id;
                max_dist = candidate_max_dist;
            }

            let curr_dist = -face_id.neg_dist;

            if max_dist - curr_dist < _eps_tol {
                let best_face = &self.faces[best_face_id.id];
                let cpts = best_face.closest_points(&self.vertices);
                return Some((cpts.0, cpts.1, best_face.normal));
            }

            let pts1 = [face.pts[0], support_point_id];
            let pts2 = [support_point_id, face.pts[1]];

            let new_faces = [
                Face::new(&self.vertices, pts1),
                Face::new(&self.vertices, pts2),
            ];

            for f in new_faces.iter() {
                if f.1 {
                    let dist = f.0.normal.dot(&f.0.proj.coords);
                    if dist < curr_dist {
                        // FIXME: if we reach this point, there were issues due to
                        // numerical errors.
                        let cpts = f.0.closest_points(&self.vertices);
                        return Some((cpts.0, cpts.1, f.0.normal));
                    }

                    if !f.0.deleted {
                        self.heap.push(FaceId::new(self.faces.len(), -dist)?);
                    }
                }

                self.faces.push(f.0.clone());
            }

            niter += 1;
            if niter > 10000 {
                //                println!("EPA did not converge after 1000 iterations… stopping the iterations.");
                return None;
            }
        }

        let best_face = &self.faces[best_face_id.id];
        let cpts = best_face.closest_points(&self.vertices);
        return Some((cpts.0, cpts.1, best_face.normal));
    }
}

fn project_origin<N: RealField>(a: &Point<N>, b: &Point<N>) -> Option<(Point<N>, [N; 2])> {
    let ab = *b - *a;
    let ap = -a.coords;
    let ab_ap = ab.dot(&ap);
    let sqnab = ab.norm_squared();

    if sqnab == na::zero() {
        return None;
    }

    let position_on_segment;

    let _eps: N = gjk::eps_tol();

    if ab_ap < -_eps || ab_ap > sqnab + _eps {
        // Voronoï region of vertex 'a' or 'b'.
        None
    } else {
        // Voronoï region of the segment interior.
        position_on_segment = ab_ap / sqnab;

        let res = *a + ab * position_on_segment;

        Some((res, [N::one() - position_on_segment, position_on_segment]))
    }
}
