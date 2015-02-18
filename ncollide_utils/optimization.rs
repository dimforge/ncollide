use std::ops::{Add, Sub, Mul};
use std::num::Float;
use rand::{self, Rand};
use na::{Inv, POrd, SquareMat, Outer, Dot, RMul};
use na;
use math::{Scalar, Vect};


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
          V: Add<V, Output = V> + Sub<V, Output = V> + Mul<V, Output = V> + Clone + Rand + POrd + Copy,
          M: Inv + Mul<V, Output = V> + Copy {
    let mut best_sol     = domain_min.clone();
    let mut best_sol_val = (*f)(domain_min);
    let domain_width     = *domain_max - *domain_min;

    for _ in range(0, num_guesses) {
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
          M: Inv + Mul<V, Output = V> + Copy {
    let mut curr = guess;

    for _ in range(0, niter) {
        let (value, mut jacobian) = (*f)(&curr);

        if !jacobian.inv_mut() {
            return (curr, false)
        }

        curr = curr - jacobian * value;
    }

    (curr, true)
}

/// Minimizes a function using the bfgs method.
pub fn minimize_with_bfgs<N, V, M, F: Fn(&V) -> N, D: Fn(&V) -> V>(
                          niter:       usize,
                          num_guesses: usize,
                          domain_min:  &V,
                          domain_max:  &V,
                          f:  &mut F,
                          df: &mut D)
                          -> (V, N)
    where N: Scalar,
          V: Vect<N> + Outer<M>,
          M: SquareMat<N, V> + Add<M, Output = M> + Sub<M, Output = M> + Clone + Copy {
    let mut best_sol     = domain_min.clone();
    let mut best_sol_val = (*f)(domain_min);
    let domain_width     = *domain_max - *domain_min;
    let ss               = BacktrackingLineSearch::new(na::one::<N>(), na::cast(0.5), na::cast(0.5), 1000);

    for _ in range(0, num_guesses) {
        let mut guess: V = rand::random();
        let shape        = na::shape(&guess);

        for i in range(0u, shape) {
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

        for _ in range(0, self.niter) {
            if fx - (*f)(&(*x + *dir * step)) >= step * t {
                break;
            }

            step = step * self.tau;
        }

        step
    }
}

/// Minimizes a function using the quasi-newton BFGS method.
pub fn bfgs<N, V, M, SS, F: Fn(&V) -> N, D: Fn(&V) -> V>(
            niter:   usize,
            ss:      &SS,
            guess:   V,
            hessian: M,
            f:       &mut F,
            df:      &mut D)
            -> V
    where N:  Scalar,
          V:  Vect<N> + Outer<M>,
          M:  SquareMat<N, V> + Add<M, Output = M> + Sub<M, Output = M> + Clone + Copy,
          SS: LineSearch<N, V> {
    let mut x  = guess;
    let mut hx = hessian;
    let mut dx = na::zero();

    if !hx.inv_mut() {
        hx = na::one();
    }

    for _ in range(0, niter) {
        let     new_dx     = (*df)(&x);
        let mut search_dir = hx.rmul(&-new_dx);

        if na::dot(&search_dir, &new_dx) >= na::zero() {
            // Not a descent direction.
            hx         = na::one();
            search_dir = -new_dx;
        }

        let _eps: N = Float::epsilon();
        let _eps    = _eps * na::cast(100.0);
        if na::sqnorm(&new_dx) <= _eps {
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
            let idenom = na::one::<N>() / denom;

            hx = hx +
                 na::outer(&step, &(step * ((denom + na::dot(&d_dx, &(hx.rmul(&d_dx)))) * idenom * idenom))) -
                 (hx * na::outer(&d_dx, &(step * idenom)) + na::outer(&step, &(d_dx * idenom)) * hx);
        }

        dx = new_dx;
    }

    x
}

#[cfg(test)]
mod test {
    use symbolic::{BivariateFn, U, V, sin, cos};
    use na::{Vec2, Mat2};
    use na;
    use super::{BacktrackingLineSearch, bfgs};

    #[test]
    fn test_bfgs_m_cos_u() {
        let f = -cos(U);
        let h = na::one::<Mat2<f64>>();
        let ss = BacktrackingLineSearch::new(1.0f64, 0.5, 0.5, 1000);

        let o = bfgs(100, &ss, Vec2::new(-0.5, -0.5), h,
                     &mut |uv| f.d0(uv.x, uv.y),
                     &mut |uv| Vec2::new(f.du(uv.x, uv.y), f.dv(uv.x, uv.y)));

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
                     Vec2::new(-0.5, -0.5),
                     h,
                     &mut |uv| f.d0(uv.x, uv.y),
                     &mut |uv| Vec2::new(f.du(uv.x, uv.y), f.dv(uv.x, uv.y)));

        println!("Minimum: f({}) = {}", o, f.d0(o.x, o.y));
        assert!(f.d0(o.x, o.y) == -2.0);
    }
}
