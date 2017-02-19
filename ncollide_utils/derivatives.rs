use alga::general::Real;

use na;

/// Computes the n-th derivative of the cosinus function.
pub fn dcos<N: Real>(n: usize, x: N) -> N {
    let n: N = na::convert(n as f64);
    (x + n * N::frac_pi_2()).cos()
}

/// Computes the n-th derivative of the sinus function.
pub fn dsin<N: Real>(n: usize, x: N) -> N {
    let n: N = na::convert(n as f64);
    (x + n * N::frac_pi_2()).sin()
}

// FIXME: this has nothing to do here…
/// Computes the binomial coefficient C^k_n ("k among n").
pub fn binom(k: usize, n: usize) -> usize {
    let mut res = 1;
    let mut n   = n;

    if k > n {
        0
    }
    else {
        for d in 1 .. k + 1 {
            res = res * n;
            res = res / d;

            n = n - 1;
        }

        res
    }
}
