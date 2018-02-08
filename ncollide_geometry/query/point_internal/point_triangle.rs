use na::{self, Real};

use utils;
use shape::Triangle;
use query::{PointProjection, PointQuery, PointQueryWithLocation};
use math::{Isometry, Point};

#[inline]
fn compute_result<P: Point>(pt: &P, proj: P) -> PointProjection<P> {
    if na::dimension::<P::Vector>() == 2 {
        PointProjection::new(*pt == proj, proj)
    } else {
        // FIXME: is this acceptable to assume the point is inside of the triangle if it is close
        // enough?
        PointProjection::new(relative_eq!(proj, *pt), proj)
    }
}

impl<P: Point, M: Isometry<P>> PointQuery<P, M> for Triangle<P> {
    #[inline]
    fn project_point(&self, m: &M, pt: &P, solid: bool) -> PointProjection<P> {
        let (projection, _) = self.project_point_with_location(m, pt, solid);
        projection
    }

    // NOTE: the default implementation of `.distance_to_point(...)` will return the error that was
    // eaten by the `::approx_eq(...)` on `project_point(...)`.
}

/// Logical description of the location of a point on a triangle.
#[derive(Copy, Clone, Debug)]
pub enum TrianglePointLocation<N: Real> {
    /// The point lies on a vertex.
    OnVertex(usize),
    /// The point lies on an edge.
    OnEdge(usize, [N; 2]),
    /// The point lies on the triangle interior.
    OnFace([N; 3]),
    /// The point lies on the triangle interior (for "solid" point queries).
    OnSolid,
}

impl<N: Real> TrianglePointLocation<N> {
    /// Returns `true` if the point is located on the relative interior of the triangle.
    pub fn is_on_face(&self) -> bool {
        if let TrianglePointLocation::OnFace(_) = *self {
            true
        } else {
            false
        }
    }
}

impl<P: Point, M: Isometry<P>> PointQueryWithLocation<P, M> for Triangle<P> {
    type Location = TrianglePointLocation<P::Real>;

    #[inline]
    fn project_point_with_location(
        &self,
        m: &M,
        pt: &P,
        solid: bool,
    ) -> (PointProjection<P>, Self::Location) {
        let a = *self.a();
        let b = *self.b();
        let c = *self.c();
        let p = m.inverse_transform_point(pt);

        let _1 = na::one::<P::Real>();

        let ab = b - a;
        let ac = c - a;
        let ap = p - a;

        let ab_ap = na::dot(&ab, &ap);
        let ac_ap = na::dot(&ac, &ap);

        if ab_ap <= na::zero() && ac_ap <= na::zero() {
            // Voronoï region of `a`.
            return (
                compute_result(pt, m.transform_point(&a)),
                TrianglePointLocation::OnVertex(0),
            );
        }

        let bp = p - b;
        let ab_bp = na::dot(&ab, &bp);
        let ac_bp = na::dot(&ac, &bp);

        if ab_bp >= na::zero() && ac_bp <= ab_bp {
            // Voronoï region of `b`.
            return (
                compute_result(pt, m.transform_point(&b)),
                TrianglePointLocation::OnVertex(1),
            );
        }

        let cp = p - c;
        let ab_cp = na::dot(&ab, &cp);
        let ac_cp = na::dot(&ac, &cp);

        if ac_cp >= na::zero() && ab_cp <= ac_cp {
            // Voronoï region of `c`.
            return (
                compute_result(pt, m.transform_point(&c)),
                TrianglePointLocation::OnVertex(2),
            );
        }

        enum ProjectionInfo<N> {
            OnAB,
            OnAC,
            OnBC,
            OnFace(N, N, N)
        }

        // Checks on which edge voronoï region the point is.
        // For 2D and 3D, it uses explicit cross/perp products that are
        // more numerically stable.
        fn stable_check_edges_voronoi<P: Point>(ab: &P::Vector, ac: &P::Vector, bc: &P::Vector,
                                                ap: &P::Vector, bp: &P::Vector, cp: &P::Vector,
                                                ab_ap: P::Real, ab_bp: P::Real,
                                                ac_ap: P::Real, ac_cp: P::Real,
                                                ac_bp: P::Real, ab_cp: P::Real)
                                               -> ProjectionInfo<P::Real> {
            match na::dimension::<P::Vector>() {
                2 => {
                    let n = utils::perp2(ab, ac);
                    let vc = n * utils::perp2(ab, ap);
                    if vc < na::zero() && ab_ap >= na::zero() && ab_bp <= na::zero() {
                        return ProjectionInfo::OnAB;
                    }

                    let vb = -n * utils::perp2(ac, cp);
                    if vb < na::zero() && ac_ap >= na::zero() && ac_cp <= na::zero() {
                        return ProjectionInfo::OnAC;
                    }

                    let va = n * utils::perp2(bc, bp);
                    if  va < na::zero() && ac_bp - ab_bp >= na::zero() && ab_cp - ac_cp >= na::zero() {
                        return ProjectionInfo::OnBC;
                    }
                    
                    return ProjectionInfo::OnFace(va, vb, vc);
                },
                3 => {
                    let n = utils::cross3(ab, ac);
                    let vc = na::dot(&n, &utils::cross3(ab, ap));
                    if vc < na::zero() && ab_ap >= na::zero() && ab_bp <= na::zero() {
                        return ProjectionInfo::OnAB;
                    }

                    let vb = -na::dot(&n, &utils::cross3(ac, cp));                    
                    if vb < na::zero() && ac_ap >= na::zero() && ac_cp <= na::zero() {
                        return ProjectionInfo::OnAC;
                    }

                    let va = na::dot(&n, &utils::cross3(bc, bp));
                    if  va < na::zero() && ac_bp - ab_bp >= na::zero() && ab_cp - ac_cp >= na::zero() {
                        return ProjectionInfo::OnBC;
                    }

                    return ProjectionInfo::OnFace(va, vb, vc);
                },
                _ => {
                    // Generic version for other dimension. May suffer from severe catastrophic cancellation issues.
                    let vc = ab_ap * ac_bp - ab_bp * ac_ap;
                    if vc < na::zero() && ab_ap >= na::zero() && ab_bp <= na::zero() {
                        return ProjectionInfo::OnAB;
                    }

                    let vb = ab_cp * ac_ap - ab_ap * ac_cp;
                    if vb < na::zero() && ac_ap >= na::zero() && ac_cp <= na::zero() {
                        return ProjectionInfo::OnAC;
                    }

                    let va = ab_bp * ac_cp - ab_cp * ac_bp;
                    if  va < na::zero() && ac_bp - ab_bp >= na::zero() && ab_cp - ac_cp >= na::zero() {
                        return ProjectionInfo::OnBC;
                    }

                    return ProjectionInfo::OnFace(va, vb, vc);
                }
            }
        }

        let bc = c - b;
        match stable_check_edges_voronoi::<P>(&ab, &ac, &bc,
                                              &ap, &bp, &cp,
                                              ab_ap, ab_bp,
                                              ac_ap, ac_cp,
                                              ac_bp, ab_cp) {
            ProjectionInfo::OnAB => {
                // Voronoï region of `ab`.
                let v = ab_ap / na::norm_squared(&ab);
                let bcoords = [_1 - v, v];

                let mut res = a;
                // NOTE: we use axpy for the GJK AnnotatedPoint trick.
                res.axpy(bcoords[1], &b, bcoords[0]);
                return (
                    compute_result(pt, m.transform_point(&res)),
                    TrianglePointLocation::OnEdge(0, bcoords),
                );
            },
            ProjectionInfo::OnAC => {
                // Voronoï region of `ac`.
                let w = ac_ap / na::norm_squared(&ac);
                let bcoords = [_1 - w, w];

                let mut res = a;
                res.axpy(bcoords[1], &c, bcoords[0]);
                return (
                    compute_result(pt, m.transform_point(&res)),
                    TrianglePointLocation::OnEdge(2, bcoords),
                );
            },
            ProjectionInfo::OnBC => {
                // Voronoï region of `bc`.
                let bc = c - b;
                let w = na::dot(&bc, &ap) / na::norm_squared(&bc);
                let bcoords = [_1 - w, w];

                let mut res = b;
                res.axpy(bcoords[1], &c, bcoords[0]);
                return (
                    compute_result(pt, m.transform_point(&res)),
                    TrianglePointLocation::OnEdge(1, bcoords),
                );
            },
            ProjectionInfo::OnFace(va, vb, vc) => {
                // Voronoï region of the face.
                if na::dimension::<P::Vector>() != 2 {
                    let denom = _1 / (va + vb + vc);
                    let v = vb * denom;
                    let w = vc * denom;
                    let bcoords = [_1 - v - w, v, w];

                    let mut res = a;
                    res.axpy(bcoords[1], &b, bcoords[0]);
                    res.axpy(bcoords[2], &c, _1);

                    return (
                        compute_result(pt, m.transform_point(&res)),
                        TrianglePointLocation::OnFace(bcoords),
                    );
                }
            }
        }

        // Special treatement if we work in 2d because in this case we really are inside of the
        // object.
        if solid {
            (
                PointProjection::new(true, *pt),
                TrianglePointLocation::OnSolid,
            )
        } else {
            // We have to project on the closest edge.

            // FIXME: this might be optimizable.
            let v = ab_ap /  (ab_ap - ab_bp); // proj on ab = a + ab * v
            let w = ac_ap / (ac_ap - ac_cp); // proj on ac = a + ac * w
            let u = (ac_bp - ab_bp) / (ac_bp - ab_bp +  ab_cp - ac_cp); // proj on bc = b + bc * u

            let bc = c - b;
            let d_ab = na::norm_squared(&ap) - (na::norm_squared(&ab) * v * v);
            let d_ac = na::norm_squared(&ap) - (na::norm_squared(&ac) * u * u);
            let d_bc = na::norm_squared(&bp) - (na::norm_squared(&bc) * w * w);

            let mut proj;
            let loc;

            if d_ab < d_ac {
                if d_ab < d_bc {
                    // ab
                    let bcoords = [_1 - v, v];
                    proj = a;
                    proj.axpy(bcoords[1], &b, bcoords[0]);
                    proj = m.transform_point(&proj);
                    loc = TrianglePointLocation::OnEdge(0, bcoords);
                } else {
                    // bc
                    let bcoords = [_1 - u, u];
                    proj = b;
                    proj.axpy(bcoords[1], &c, bcoords[0]);
                    proj = m.transform_point(&proj);
                    loc = TrianglePointLocation::OnEdge(1, bcoords);
                }
            } else {
                if d_ac < d_bc {
                    // ac
                    let bcoords = [_1 - w, w];
                    proj = a;
                    proj.axpy(bcoords[1], &c, bcoords[0]);
                    proj = m.transform_point(&proj);
                    loc = TrianglePointLocation::OnEdge(2, bcoords);
                } else {
                    // bc
                    let bcoords = [_1 - u, u];
                    proj = b;
                    proj.axpy(bcoords[1], &c, bcoords[0]);
                    proj = m.transform_point(&proj);
                    loc = TrianglePointLocation::OnEdge(1, bcoords);
                }
            }

            (PointProjection::new(true, proj), loc)
        }
    }
}
