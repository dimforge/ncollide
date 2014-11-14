use na::Transform;
use na;
use shape::Triangle;
use point::{LocalPointQuery, PointQuery};
use math::{Scalar, Point, Vect};


impl<N, P, V> LocalPointQuery<N, P> for Triangle<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    #[inline]
    fn project_point(&self, pt: &P, solid: bool) -> P {
        /*
         * This comes from the book `Real Time Collision Detection`.
         * This is a trivial Voronoï region based approach, except that great care has been taken
         * to avoid cross products (which is good for the genericity here).
         *
         * We keep the original (somehow, obscure) notations for future reference.
         */
        let a = self.a().clone();
        let b = self.b().clone();
        let c = self.c().clone();
        let p = pt.clone();

        let ab = b - a;
        let ac = c - a;
        let ap = p - a;

        let d1 = na::dot(&ab, &ap);
        let d2 = na::dot(&ac, &ap);

        if d1 <= na::zero() && d2 <= na::zero() {
            // Voronoï region of `a`.
            return a;
        }

        let bp = p - b;
        let d3 = na::dot(&ab, &bp);
        let d4 = na::dot(&ac, &bp);

        if d3 >= na::zero() && d4 <= d3 {
            // Voronoï region of `b`.
            return b;
        }

        let vc = d1 * d4 - d3 * d2;
        if vc <= na::zero() && d1 >= na::zero() && d3 <= na::zero() {
            // Voronoï region of `ab`.
            let v = d1 / (d1 - d3);
            return a + ab * v;
        }

        let cp = p - c;
        let d5 = na::dot(&ab, &cp);
        let d6 = na::dot(&ac, &cp);

        if d6 >= na::zero() && d5 <= d6 {
            // Voronoï region of `c`.
            return c;
        }

        let vb = d5 * d2 - d1 * d6;

        if vb <= na::zero() && d2 >= na::zero() && d6 <= na::zero() {
            // Voronoï region of `ac`.
            let w = d2 / (d2 - d6);
            return a + ac * w;
        }

        let va = d3 * d6 - d5 * d4;
        if va <= na::zero() && d4 - d3 >= na::zero() && d5 - d6 >= na::zero() {
            // Voronoï region of `bc`.
            let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return b + (c - b) * w;
        }

        // Voronoï region of the face.
        if na::dim::<P>() != 2 {
            let denom = na::one::<N>() / (va + vb + vc);
            let v = vb * denom;
            let w = vc * denom;

            a + ab * v + ac * w
        }
        else {
            // Special treatement if we work in 2d because in this case we really are inside of the
            // object.
            if solid {
                p
            }
            else {
                // We have to project on the closest edge.

                // FIXME: this might be optimizable.
                let v = d1 / (d1 - d3);                      // proj on ab = a + ab * v
                let w = d2 / (d2 - d6);                      // proj on ac = a + ac * w
                let u = (d4 - d3) / ((d4 - d3) + (d5 - d6)); // proj on bc = b + bc * u

                let bc = c - b;
                let d_ab = na::sqnorm(&ap) - (na::sqnorm(&ab) * v * v);
                let d_ac = na::sqnorm(&ap) - (na::sqnorm(&ac) * u * u);
                let d_bc = na::sqnorm(&bp) - (na::sqnorm(&bc) * w * w);

                if d_ab < d_ac {
                    if d_ab < d_bc {
                        // ab
                        a + ab * v
                    }
                    else {
                        // bc
                        b + bc * u
                    }
                }
                else {
                    if d_ac < d_bc {
                        // ac
                        a + ac * w
                    }
                    else {
                        // bc
                        b + bc * u
                    }
                }
            }
        }
    }

    #[inline]
    fn distance_to_point(&self, pt: &P) -> N {
        na::dist(pt, &self.project_point(pt, true))
    }

    #[inline]
    fn contains_point(&self, pt: &P) -> bool {
        na::approx_eq(&self.distance_to_point(pt), &na::zero())
    }
}

impl<N, P, V, M> PointQuery<N, P, M> for Triangle<P>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N>,
          M: Transform<P> {
}
