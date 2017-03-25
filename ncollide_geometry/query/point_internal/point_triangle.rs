use na;
use shape::Triangle;
use query::{PointQuery, PointProjection};
use math::{Point, Isometry};

#[inline]
fn compute_result<P: Point>(pt: &P, proj: P) -> PointProjection<P> {
   if na::dimension::<P::Vector>() == 2 {
       PointProjection::new(*pt == proj, proj, ())
   }
   else {
       // FIXME: is this acceptable to assume the point is inside of the triangle if it is close
       // enough?
       PointProjection::new(relative_eq!(proj, *pt), proj, ())
   }
}

impl<P: Point, M: Isometry<P>> PointQuery<P, M> for Triangle<P> {
    #[inline]
    fn project_point(&self, m: &M, pt: &P, solid: bool) -> PointProjection<P> {
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

        let ab = b - a;
        let ac = c - a;
        let ap = p - a;

        let d1 = na::dot(&ab, &ap);
        let d2 = na::dot(&ac, &ap);

        if d1 <= na::zero() && d2 <= na::zero() {
            // Voronoï region of `a`.
            return compute_result(pt, m.transform_point(&a));
        }

        let bp = p - b;
        let d3 = na::dot(&ab, &bp);
        let d4 = na::dot(&ac, &bp);

        if d3 >= na::zero() && d4 <= d3 {
            // Voronoï region of `b`.
            return compute_result(pt, m.transform_point(&b));
        }

        let vc = d1 * d4 - d3 * d2;
        if vc <= na::zero() && d1 >= na::zero() && d3 <= na::zero() {
            // Voronoï region of `ab`.
            let v = d1 / (d1 - d3);
            return compute_result(pt, m.transform_point(&(a + ab * v)));
        }

        let cp = p - c;
        let d5 = na::dot(&ab, &cp);
        let d6 = na::dot(&ac, &cp);

        if d6 >= na::zero() && d5 <= d6 {
            // Voronoï region of `c`.
            return compute_result(pt, m.transform_point(&c));
        }

        let vb = d5 * d2 - d1 * d6;

        if vb <= na::zero() && d2 >= na::zero() && d6 <= na::zero() {
            // Voronoï region of `ac`.
            let w = d2 / (d2 - d6);
            return compute_result(pt, m.transform_point(&(a + ac * w)));
        }

        let va = d3 * d6 - d5 * d4;
        if va <= na::zero() && d4 - d3 >= na::zero() && d5 - d6 >= na::zero() {
            // Voronoï region of `bc`.
            let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return compute_result(pt, m.transform_point(&(b + (c - b) * w)));
        }

        // Voronoï region of the face.
        if na::dimension::<P::Vector>() != 2 {
            let denom = na::one::<P::Real>() / (va + vb + vc);
            let v = vb * denom;
            let w = vc * denom;

            return compute_result(pt, m.transform_point(&(a + ab * v + ac * w)));
        }
        else {
            // Special treatement if we work in 2d because in this case we really are inside of the
            // object.
            if solid {
                PointProjection::new(true, *pt, ())
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

                let proj;

                if d_ab < d_ac {
                    if d_ab < d_bc {
                        // ab
                        proj = m.transform_point(&(a + ab * v));
                    }
                    else {
                        // bc
                        proj = m.transform_point(&(b + bc * u));
                    }
                }
                else {
                    if d_ac < d_bc {
                        // ac
                        proj = m.transform_point(&(a + ac * w));
                    }
                    else {
                        // bc
                        proj = m.transform_point(&(b + bc * u));
                    }
                }

                PointProjection::new(true, proj, ())
            }
        }
    }

    // NOTE: the default implementation of `.distance_to_point(...)` will return the error that was
    // eaten by the `::approx_eq(...)` on `project_point(...)`.
}
