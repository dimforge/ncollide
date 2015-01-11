use na::BaseFloat;
use na;
use math::Scalar;

/// Computes the n-th derivative of the cosinus function.
pub fn dcos<N: Scalar>(n: usize, x: N) -> N {
    let n: N = na::cast(n as f64);
    (x + n * BaseFloat::frac_pi_2()).cos()
}

/// Computes the n-th derivative of the sinus function.
pub fn dsin<N: Scalar>(n: usize, x: N) -> N {
    let n: N = na::cast(n as f64);
    (x + n * BaseFloat::frac_pi_2()).sin()
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
        for d in range(1, k + 1) {
            res = res * n;
            res = res / d;

            n = n - 1;
        }

        res
    }
}
