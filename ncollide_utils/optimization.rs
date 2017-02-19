use std::ops::{Add, Sub, Mul};
use rand::{self, Rand};
use na::{Inverse, PartialOrder, SquareMatrix, Outer, Dot};
use na;
use math::{Scalar, Vector, FloatError};


// FIXME: implement a proper metaheuristic.
/// Maximizes a real function using the Newton method.
pub fn maximize_with_newton<N, V, M, F: Fn(&V) -> N, D: Fn(&V) -> (V, M)>(
                            niter:       usize,
                            num_guesses: usize,
                            domain_min:  &V,
                            domain_max:  &V,
                            f:  &mut F,
                            df: &mut D)
                            -> (V, N)
    where N: Scalar,
          V: Add<V, Output = V> + Sub<V, Output = V> + Mul<V, Output = V> + Clone + Rand + PartialOrder + Copy,
          M: Inverse + Mul<V, Output = V> + Copy {
    let mut best_sol     = domain_min.clone();
    let mut best_sol_val = (*f)(domain_min);
    let domain_width     = *domain_max - *domain_min;

    for _ in 0 .. num_guesses {
        let guess: V = rand::random();
        let guess    = *domain_min + domain_width * guess; // FIXME: let the user pass a random generator?
        let (arg, _) = newton(niter, guess, df);

        if na::partial_le(&arg, domain_max) && na::partial_ge(&arg, domain_min) {
            let val = (*f)(&arg);

            if val > best_sol_val {
                best_sol_val = val;
                best_sol     = arg;

            }
        }
    }

    (best_sol, best_sol_val)
}

/// Finds the root of a function using the Newton method.
pub fn newton<V, M, F: Fn(&V) -> (V, M)>(niter: usize, guess: V, f: &mut F) -> (V, bool)
    where V: Sub<V, Output = V>,
          M: Inverse + Mul<V, Output = V> + Copy {
    let mut curr = guess;

    for _ in 0 .. niter {
        let (value, mut jacobian) = (*f)(&curr);

        if !jacobian.inverse_mut() {
            return (curr, false)
        }

        curr = curr - jacobian * value;
    }

    (curr, true)
}

/// Minimizes a function using the bfgs method.
pub fn minimize_with_bfgs<V, F, D>(niter:       usize,
                                   num_guesses: usize,
                                   domain_min:  &V,
                                   domain_max:  &V,
                                   f:           &mut F,
                                   df:          &mut D)
                                   -> (V, V::Scalar)
    where V: Vector + Outer + Mul<<V as Outer>::OuterProductType, Output = V>,
          F: Fn(&V) -> V::Scalar,
          D: Fn(&V) -> V,
          V::OuterProductType: SquareMatrix<<V as Vector>::Scalar, V> +
                               Add<<V as Outer>::OuterProductType, Output = <V as Outer>::OuterProductType> +
                               Sub<<V as Outer>::OuterProductType, Output = <V as Outer>::OuterProductType> +
                               Clone + Copy {
    let mut best_sol     = domain_min.clone();
    let mut best_sol_val = (*f)(domain_min);
    let domain_width     = *domain_max - *domain_min;
    let ss               = BacktrackingLineSearch::new(na::one::<V::Scalar>(), na::convert(0.5), na::convert(0.5), 1000);

    for _ in 0 .. num_guesses {
        let mut guess: V = rand::random();
        let shape        = na::shape(&guess);

        for i in 0usize .. shape {
            let inbound_guess = domain_min[i] + guess[i] * domain_width[i];
            guess[i] = inbound_guess;
        }

        let arg = bfgs(niter, &ss, guess, na::one(), f, df);

        if na::partial_le(&arg, domain_max) && na::partial_ge(&arg, domain_min) {
            let val = (*f)(&arg);

            if val < best_sol_val {
                best_sol_val = val;
                best_sol     = arg;
            }
        }
    }

    (best_sol, best_sol_val)
}

/// Trait for line search methods.
pub trait LineSearch<N, V> {
    /// Gets a near-optimal step size for the next descent.
    fn step_size<F: Fn(&V) -> N>(&self, f: &mut F, df: &V, x: &V, dir: &V) -> N;
}

/// The backtracking line search method.
pub struct BacktrackingLineSearch<N> {
    alpha: N,
    tau:   N,
    c:     N,
    niter: usize
}

impl<N> BacktrackingLineSearch<N> {
    /// Creates a new backtracking line search methods.
    pub fn new(alpha: N, tau: N, c: N, niter: usize) -> BacktrackingLineSearch<N> {
        BacktrackingLineSearch {
            alpha: alpha,
            tau:   tau,
            c:     c,
            niter: niter
        }
    }
}

impl<N, V> LineSearch<N, V> for BacktrackingLineSearch<N>
    where N: Scalar,
          V: Dot<N> + Add<V, Output = V> + Mul<N, Output = V> + Copy {
    fn step_size<F: Fn(&V) -> N>(&self, f: &mut F, df: &V, x: &V, dir: &V) -> N {
        let     t    = -self.c * na::dot(df, dir);
        let     fx   = (*f)(x);
        let mut step = self.alpha.clone();

        for _ in 0 .. self.niter {
            if fx - (*f)(&(*x + *dir * step)) >= step * t {
                break;
            }

            step = step * self.tau;
        }

        step
    }
}

/// Minimizes a function using the quasi-newton BFGS method.
pub fn bfgs<V, SS, F, D>(niter:   usize,
                         ss:      &SS,
                         guess:   V,
                         hessian: V::OuterProductType,
                         f:       &mut F,
                         df:      &mut D)
                         -> V
    where V: Vector + Outer + Mul<<V as Outer>::OuterProductType, Output = V>,
          F: Fn(&V) -> V::Scalar,
          D: Fn(&V) -> V,
          V::OuterProductType: SquareMatrix<<V as Vector>::Scalar, V> +
                               Add<<V as Outer>::OuterProductType, Output = <V as Outer>::OuterProductType> +
                               Sub<<V as Outer>::OuterProductType, Output = <V as Outer>::OuterProductType> +
                               Clone + Copy,
          SS: LineSearch<V::Scalar, V> {
    let mut x  = guess;
    let mut hx = hessian;
    let mut dx = na::zero();

    if !hx.inverse_mut() {
        hx = na::one();
    }

    for _ in 0 .. niter {
        let     new_dx     = (*df)(&x);
        let mut search_dir = hx * (-new_dx);

        if na::dot(&search_dir, &new_dx) >= na::zero() {
            // Not a descent direction.
            hx         = na::one();
            search_dir = -new_dx;
        }

        let _eps: V::Scalar = FloatError::epsilon();
        let _eps = _eps * na::convert(100.0);
        if na::norm_squared(&new_dx) <= _eps {
            break;
        }

        let alpha = ss.step_size(f, &new_dx, &x, &search_dir);
        let step  = search_dir * alpha;

        if alpha * alpha <= _eps {
            break;
        }

        x = x + step;

        let d_dx  = new_dx - dx;
        let denom = na::dot(&step, &d_dx);

        if denom * denom <= _eps {
            hx = na::one()
        }
        else {
            let idenom = na::one::<V::Scalar>() / denom;

            hx = hx +
                 na::outer(&step, &(step * ((denom + na::dot(&d_dx, &(hx * d_dx))) * idenom * idenom))) -
                 (hx * na::outer(&d_dx, &(step * idenom)) + na::outer(&step, &(d_dx * idenom)) * hx);
        }

        dx = new_dx;
    }

    x
}

#[cfg(test)]
mod test {
    use symbolic::{BivariateFn, U, V, sin, cos};
    use na::{Vector2, Mat2};
    use na;
    use super::{BacktrackingLineSearch, bfgs};

    #[test]
    fn test_bfgs_m_cos_u() {
        let f = -cos(U);
        let h = na::one::<Mat2<f64>>();
        let ss = BacktrackingLineSearch::new(1.0f64, 0.5, 0.5, 1000);

        let o = bfgs(100, &ss, Vector2::new(-0.5, -0.5), h,
                     &mut |uv| f.d0(uv.x, uv.y),
                     &mut |uv| Vector2::new(f.du(uv.x, uv.y), f.dv(uv.x, uv.y)));

        println!("Minimum: f({}) = {}", o, f.d0(o.x, o.y));
        assert!(f.d0(o.x, o.y) == -1.0);
    }

    #[test]
    fn test_bfgs_m_cos_u_m_sin_v() {
        let f  = -cos(U) - sin(V);
        let h  = na::one::<Mat2<f64>>();
        let ss = BacktrackingLineSearch::new(1.0f64, 0.5, 0.5, 1000);

        let o = bfgs(10000,
                     &ss,
                     Vector2::new(-0.5, -0.5),
                     h,
                     &mut |uv| f.d0(uv.x, uv.y),
                     &mut |uv| Vector2::new(f.du(uv.x, uv.y), f.dv(uv.x, uv.y)));

        println!("Minimum: f({}) = {}", o, f.d0(o.x, o.y));
        assert!(f.d0(o.x, o.y) == -2.0);
    }
}
