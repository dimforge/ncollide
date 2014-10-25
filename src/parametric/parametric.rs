// FIXME: support more than rectangular surfaces?

use math::Scalar;

/// Trait implemented by differentiable parametric surfaces.
///
/// The parametrization space is assumed to be `[0.0, 1.0] x [0.0, 1.0]`.
pub trait ParametricSurface<N: Scalar, P, V> {
    // FIXME: rename those d0, du, etc. ? (just like in the `symbolic` module.
    /// Evaluates the parametric surface.
    fn at(&self, u: N, v: N) -> P;

    /// Evaluates the surface derivative wrt. `u`.
    fn at_u(&self, u: N, v: N) -> V;

    /// Evaluates the surface derivative wrt. `v`.
    fn at_v(&self, u: N, v: N) -> V;

    /// Evaluates the surface second derivative wrt. `u`.
    fn at_uu(&self, u: N, v: N) -> V;

    /// Evaluates the surface second derivative wrt. `v`.
    fn at_vv(&self, u: N, v: N) -> V;

    /// Evaluates the surface second derivative wrt. `u` and `v`.
    fn at_uv(&self, u: N, v: N) -> V;

    /// Evaluates the parametric surface and its first derivatives.
    ///
    /// Returns (S(u, v), dS / du, dS / dv)
    #[inline]
    fn at_u_v(&self, u: N, v: N) -> (P, V, V) {
        (self.at(u, v), self.at_u(u, v), self.at_v(u, v))
    }

    /// Evaluates the parametric surface and its first and second derivatides.
    ///
    /// Returns (S(u, v), dS / du, dS / dv, d²S / du², d²S / dv², d²S / dudv)
    #[inline]
    fn at_u_v_uu_vv_uv(&self, u: N, v: N) -> (P, V, V, V, V, V) {
        (self.at(u, v), self.at_u(u, v), self.at_v(u, v), self.at_uu(u, v), self.at_vv(u, v), self.at_uv(u, v))
    }

    /// Evaluates the parametric surface and its derivative wrt. `u` `n` times and wrt. `v` `k` times.
    ///
    /// Returns d^(n + k) S(u, v) / (du^n dk^n).
    fn at_uv_nk(&self, u: N, v: N, n: uint, k: uint) -> V;
}
