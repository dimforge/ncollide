/// Computes the n-th derivative of the cosinus function.
pub fn dcos<N: FloatMath>(n: uint, x: N) -> N {
    let value;

    if n % 2 == 0 {
        value = x.cos();
    }
    else {
        value = x.sin();
    }

    if ((n + 4 - 1) / 2) % 2 == 0 { // the `+ 4` will prevent problems with negativity.
        -value
    }
    else {
        value
    }
}

/// Computes the n-th derivative of the sinus function.
pub fn dsin<N: FloatMath>(n: uint, x: N) -> N {
    let value;

    if n % 2 == 0 {
        value = x.sin();
    }
    else {
        value = x.cos();
    }

    if ((n + 4 - 2) / 2) % 2 == 0 { // the `+ 4` will prevent problems with negativity.
        -value
    }
    else {
        value
    }
}
