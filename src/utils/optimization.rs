use std::num::{Zero, One};
use std::cmp;
use std::rand::Rand;
use std::rand;
use nalgebra::na::{Inv, PartialOrd, Mat, Outer, Cast, Dot, RMul, ScalarMul, Indexable, FloatVec};
use nalgebra::na;


// FIXME: implement a proper metaheuristic.
/// Maximizes a real function using the Newton method.
pub fn maximize_with_newton<N: Float,
                            V: Add<V, V> + Sub<V, V> + Mul<V, V> + Clone + Rand + PartialOrd,
                            M: Inv + Mul<V, V>>(
                            niter:       uint,
                            num_guesses: uint,
                            domain_min:  &V,
                            domain_max:  &V,
                            f:  &mut |&V| -> N,
                            df: &mut |&V| -> (V, M))
                            -> (V, N) {
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
pub fn newton<V: Sub<V, V>, M: Inv + Mul<V, V>>(
              niter: uint, guess: V, f: &mut |&V| -> (V, M)) -> (V, bool) {
    let mut curr = guess;

    for _ in range(0, niter) {
        let (value, mut jacobian) = (*f)(&curr);

        if !jacobian.inv() {
            return (curr, false)
        }

        curr = curr - jacobian * value;
    }

    (curr, true)
}

/// Minimizes a function using the bfgs method.
pub fn minimize_with_bfgs<N: cmp::PartialOrd + Zero + Cast<f64> + Float + Rand,
                          V: FloatVec<N> + Outer<M> + PartialOrd + Clone + Indexable<uint, N> + Rand,
                          M: Mat<V, V> + Mul<M, M> + Add<M, M> + Sub<M, M> + Inv + One + Clone + ScalarMul<N>>(
                          niter:       uint,
                          num_guesses: uint,
                          domain_min:  &V,
                          domain_max:  &V,
                          f:  &mut |&V| -> N,
                          df: &mut |&V| -> V)
                          -> (V, N) {
    let mut best_sol     = domain_min.clone();
    let mut best_sol_val = (*f)(domain_min);
    let domain_width     = *domain_max - *domain_min;
    let ss               = BacktrackingLineSearch::new(na::one::<N>(), na::cast(0.5), na::cast(0.5), 1000);

    for _ in range(0, num_guesses) {
        let mut guess: V = rand::random();
        let shape        = guess.shape();

        for i in range(0u, shape) {
            let inbound_guess = domain_min.at(i) + guess.at(i) * domain_width.at(i);
            guess.set(i, inbound_guess);
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
    fn step_size(&self, f: &mut |&V| -> N, df: &V, x: &V, dir: &V) -> N;
}

/// The backtracking line search method.
pub struct BacktrackingLineSearch<N> {
    alpha: N,
    tau:   N,
    c:     N,
    niter: uint
}

impl<N> BacktrackingLineSearch<N> {
    /// Creates a new backtracking line search methods.
    pub fn new(alpha: N, tau: N, c: N, niter: uint) -> BacktrackingLineSearch<N> {
        BacktrackingLineSearch {
            alpha: alpha,
            tau:   tau,
            c:     c,
            niter: niter
        }
    }
}

impl<N: Neg<N> + Sub<N, N> + Mul<N, N> + cmp::PartialOrd + Clone, V: Dot<N> + Add<V, V> + Mul<N, V>>
LineSearch<N, V> for BacktrackingLineSearch<N> {
    fn step_size(&self, f: &mut |&V| -> N, df: &V, x: &V, dir: &V) -> N {
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
pub fn bfgs<N: cmp::PartialOrd + Zero + Cast<f64> + Float,
            V: FloatVec<N> + Outer<M> + Clone,
            M: Mat<V, V> + Mul<M, M> + Add<M, M> + Sub<M, M> + Inv + One + Clone + ScalarMul<N>,
            SS: LineSearch<N, V>>(
            niter:   uint,
            ss:      &SS,
            guess:   V,
            hessian: M,
            f:       &mut |&V| -> N,
            df:      &mut |&V| -> V)
            -> V {
    let mut x  = guess;
    let mut hx = hessian;
    let mut dx = na::zero();

    if !hx.inv() {
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
        let step  = search_dir.mul_s(&alpha);

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
                 na::outer(&step, &step).mul_s(&((denom + na::dot(&d_dx, &(hx.rmul(&d_dx)))) * idenom * idenom)) -
                 (hx * na::outer(&d_dx, &step) + na::outer(&step, &d_dx) * hx).mul_s(&idenom);
        }

        dx = new_dx;
    }

    x
}

#[cfg(test)]
mod test {
    use utils::symbolic::{BivariateFn, U, V, sin, cos};
    use nalgebra::na::{Vec2, Mat2};
    use nalgebra::na;
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
