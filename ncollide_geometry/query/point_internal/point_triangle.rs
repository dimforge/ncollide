use na::{self, Real};
use shape::Triangle;
use query::{PointQuery, PointProjection, PointQueryWithLocation};
use math::{Point, Isometry};

#[inline]
fn compute_result<P: Point>(pt: &P, proj: P) -> PointProjection<P> {
   if na::dimension::<P::Vector>() == 2 {
       PointProjection::new(*pt == proj, proj)
   }
   else {
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
    /// The point lies on a vertex.
    OnEdge(usize, [N; 2]),
    /// The point lies on the triangle interior.
    OnFace([N; 3]),
    /// The point lies on the triangle interior (for "solid" point queries).
    OnSolid
}

impl<N: Real> TrianglePointLocation<N> {
    /// Returns `true` if the point is located on the relative interior of the triangle.
    pub fn is_on_face(&self) -> bool {
        if let TrianglePointLocation::OnFace(_) = *self {
            true
        }
        else {
            false
        }
    }
}

impl<P: Point, M: Isometry<P>> PointQueryWithLocation<P, M> for Triangle<P> {
    type Location = TrianglePointLocation<P::Real>;

    #[inline]
    fn project_point_with_location(&self, m: &M, pt: &P, solid: bool)
        -> (PointProjection<P>, Self::Location)
    {
        /*
         * This comes from the book `Real Time Collision Detection`.
         * This is actually a trivial Voronoï region based approach, except that great care has
         * been taken to avoid cross products (which is good for the genericity here).
         *
         * We keep the original (somehow, obscure like d1 ... d6) notations for future reference.
         */
        let a = *self.a();
        let b = *self.b();
        let c = *self.c();
        let p = m.inverse_transform_point(pt);

        let _1 = na::one::<P::Real>();

        let ab = b - a;
        let ac = c - a;
        let ap = p - a;

        let d1 = na::dot(&ab, &ap);
        let d2 = na::dot(&ac, &ap);

        if d1 <= na::zero() && d2 <= na::zero() {
            // Voronoï region of `a`.
            return (compute_result(pt, m.transform_point(&a)), TrianglePointLocation::OnVertex(0));
        }

        let bp = p - b;
        let d3 = na::dot(&ab, &bp);
        let d4 = na::dot(&ac, &bp);

        if d3 >= na::zero() && d4 <= d3 {
            // Voronoï region of `b`.
            return (compute_result(pt, m.transform_point(&b)), TrianglePointLocation::OnVertex(1));
        }

        let vc = d1 * d4 - d3 * d2;
        if vc <= na::zero() && d1 >= na::zero() && d3 <= na::zero() {
            // Voronoï region of `ab`.
            let v       = d1 / (d1 - d3);
            let bcoords = [ _1 - v, v ];

            let mut res = a;
            // NOTE: we use axpy for the GJK AnnotatedPoint trick.
            res.axpy(bcoords[1], &b, bcoords[0]);
            return (compute_result(pt, m.transform_point(&res)), TrianglePointLocation::OnEdge(0, bcoords));
        }

        let cp = p - c;
        let d5 = na::dot(&ab, &cp);
        let d6 = na::dot(&ac, &cp);

        if d6 >= na::zero() && d5 <= d6 {
            // Voronoï region of `c`.
            return (compute_result(pt, m.transform_point(&c)), TrianglePointLocation::OnVertex(2));
        }

        let vb = d5 * d2 - d1 * d6;

        if vb <= na::zero() && d2 >= na::zero() && d6 <= na::zero() {
            // Voronoï region of `ac`.
            let w       = d2 / (d2 - d6);
            let bcoords = [ _1 - w, w ];

            let mut res = a;
            res.axpy(bcoords[1], &c, bcoords[0]);
            return (compute_result(pt, m.transform_point(&res)), TrianglePointLocation::OnEdge(2, bcoords));
        }

        let va = d3 * d6 - d5 * d4;
        if va <= na::zero() && d4 - d3 >= na::zero() && d5 - d6 >= na::zero() {
            // Voronoï region of `bc`.
            let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            let bcoords = [ _1 - w, w ];

            let mut res = b;
            res.axpy(bcoords[1], &c, bcoords[0]);
            return (compute_result(pt, m.transform_point(&res)), TrianglePointLocation::OnEdge(1, bcoords));
        }

        // Voronoï region of the face.
        if na::dimension::<P::Vector>() != 2 {
            let denom = _1 / (va + vb + vc);
            let v = vb * denom;
            let w = vc * denom;
            let bcoords = [ _1 - v - w, v, w ];

            let mut res = a;
            res.axpy(bcoords[1], &b, bcoords[0]);
            res.axpy(bcoords[2], &c, _1);

            return (compute_result(pt, m.transform_point(&res)), TrianglePointLocation::OnFace(bcoords));
        }
        else {
            // Special treatement if we work in 2d because in this case we really are inside of the
            // object.
            if solid {
                (PointProjection::new(true, *pt), TrianglePointLocation::OnSolid)
            }
            else {
                // We have to project on the closest edge.

                // FIXME: this might be optimizable.
                let v = d1 / (d1 - d3);                      // proj on ab = a + ab * v
                let w = d2 / (d2 - d6);                      // proj on ac = a + ac * w
                let u = (d4 - d3) / ((d4 - d3) + (d5 - d6)); // proj on bc = b + bc * u

                let bc = c - b;
                let d_ab = na::norm_squared(&ap) - (na::norm_squared(&ab) * v * v);
                let d_ac = na::norm_squared(&ap) - (na::norm_squared(&ac) * u * u);
                let d_bc = na::norm_squared(&bp) - (na::norm_squared(&bc) * w * w);

                let mut proj;
                let loc;

                if d_ab < d_ac {
                    if d_ab < d_bc {
                        // ab
                        let bcoords = [ _1 - v, v ];
                        proj = a;
                        proj.axpy(bcoords[1], &b, bcoords[0]);
                        proj = m.transform_point(&proj);
                        loc  = TrianglePointLocation::OnEdge(0, bcoords);
                    }
                    else {
                        // bc
                        let bcoords = [ _1 - u, u ];
                        proj = b;
                        proj.axpy(bcoords[1], &c, bcoords[0]);
                        proj = m.transform_point(&proj);
                        loc  = TrianglePointLocation::OnEdge(1, bcoords);
                    }
                }
                else {
                    if d_ac < d_bc {
                        // ac
                        let bcoords = [ _1 - w, w ];
                        proj = a;
                        proj.axpy(bcoords[1], &c, bcoords[0]);
                        proj = m.transform_point(&proj);
                        loc  = TrianglePointLocation::OnEdge(2, bcoords);
                    }
                    else {
                        // bc
                        let bcoords = [ _1 - u, u ];
                        proj = b;
                        proj.axpy(bcoords[1], &c, bcoords[0]);
                        proj = m.transform_point(&proj);
                        loc  = TrianglePointLocation::OnEdge(1, bcoords);
                    }
                }

                (PointProjection::new(true, proj), loc)
            }
        }
    }
}
