use na::Transform;
use na;
use entities::shape::Triangle;
use point::PointQuery;
use math::{Point, Vector};


impl<P, M> PointQuery<P, M> for Triangle<P>
    where P: Point,
          M: Transform<P> {
    #[inline]
    fn project_point(&self, m: &M, pt: &P, solid: bool) -> P {
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
        let p = m.inverse_transform(pt);

        let ab = b - a;
        let ac = c - a;
        let ap = p - a;

        let d1 = na::dot(&ab, &ap);
        let d2 = na::dot(&ac, &ap);

        if d1 <= na::zero() && d2 <= na::zero() {
            // Voronoï region of `a`.
            return m.transform(&a);
        }

        let bp = p - b;
        let d3 = na::dot(&ab, &bp);
        let d4 = na::dot(&ac, &bp);

        if d3 >= na::zero() && d4 <= d3 {
            // Voronoï region of `b`.
            return m.transform(&b);
        }

        let vc = d1 * d4 - d3 * d2;
        if vc <= na::zero() && d1 >= na::zero() && d3 <= na::zero() {
            // Voronoï region of `ab`.
            let v = d1 / (d1 - d3);
            return m.transform(&(a + ab * v));
        }

        let cp = p - c;
        let d5 = na::dot(&ab, &cp);
        let d6 = na::dot(&ac, &cp);

        if d6 >= na::zero() && d5 <= d6 {
            // Voronoï region of `c`.
            return m.transform(&c);
        }

        let vb = d5 * d2 - d1 * d6;

        if vb <= na::zero() && d2 >= na::zero() && d6 <= na::zero() {
            // Voronoï region of `ac`.
            let w = d2 / (d2 - d6);
            return m.transform(&(a + ac * w));
        }

        let va = d3 * d6 - d5 * d4;
        if va <= na::zero() && d4 - d3 >= na::zero() && d5 - d6 >= na::zero() {
            // Voronoï region of `bc`.
            let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            return m.transform(&(b + (c - b) * w));
        }

        // Voronoï region of the face.
        if na::dimension::<P>() != 2 {
            let denom = na::one::<<P::Vect as Vector>::Scalar>() / (va + vb + vc);
            let v = vb * denom;
            let w = vc * denom;

            m.transform(&(a + ab * v + ac * w))
        }
        else {
            // Special treatement if we work in 2d because in this case we really are inside of the
            // object.
            if solid {
                pt.clone()
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

                if d_ab < d_ac {
                    if d_ab < d_bc {
                        // ab
                        m.transform(&(a + ab * v))
                    }
                    else {
                        // bc
                        m.transform(&(b + bc * u))
                    }
                }
                else {
                    if d_ac < d_bc {
                        // ac
                        m.transform(&(a + ac * w))
                    }
                    else {
                        // bc
                        m.transform(&(b + bc * u))
                    }
                }
            }
        }
    }

    #[inline]
    fn distance_to_point(&self, m: &M, pt: &P, solid: bool) -> <P::Vect as Vector>::Scalar {
        na::distance(pt, &self.project_point(m, pt, solid))
    }

    #[inline]
    fn contains_point(&self, m: &M, pt: &P) -> bool {
        na::approx_eq(&self.distance_to_point(m, pt, true), &na::zero())
    }
}
