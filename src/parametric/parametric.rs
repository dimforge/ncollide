use math::{Scalar, Vect};


// FIXME: support more than rectangular surfaces?
/// Trait implemented by C2 parametric surfaces.
///
/// The parametrization space is assumed to be `[0.0, 1.0] x [0.0, 1.0]`.
pub trait ParametricSurface {
    /// Evaluates the parametric surface.
    fn at(&self, u: Scalar, v: Scalar) -> Vect;

    /// Evaluates the surface derivative wrt. `u`.
    fn at_u(&self, u: Scalar, v: Scalar) -> Vect;

    /// Evaluates the surface derivative wrt. `v`.
    fn at_v(&self, u: Scalar, v: Scalar) -> Vect;

    /// Evaluates the surface second derivative wrt. `u`.
    fn at_uu(&self, u: Scalar, v: Scalar) -> Vect;

    /// Evaluates the surface second derivative wrt. `v`.
    fn at_vv(&self, u: Scalar, v: Scalar) -> Vect;

    /// Evaluates the surface second derivative wrt. `u` and `v`.
    fn at_uv(&self, u: Scalar, v: Scalar) -> Vect;

    /// Evaluates the parametric surface and its first derivatives.
    ///
    /// Returns (S(u, v), dS / du, dS / dv)
    fn at_u_v(&self, u: Scalar, v: Scalar) -> (Vect, Vect, Vect) {
        (self.at(u, v), self.at_u(u, v), self.at_v(u, v))
    }

    /// Evaluates the parametric surface and its first and second derivatides.
    ///
    /// Returns (S(u, v), dS / du, dS / dv, d²S / du², d²S / dv², d²S / dudv)
    fn at_u_v_uu_vv_uv(&self, u: Scalar, v: Scalar) -> (Vect, Vect, Vect, Vect, Vect, Vect) {
        (self.at(u, v), self.at_u(u, v), self.at_v(u, v), self.at_uu(u, v), self.at_vv(u, v), self.at_uv(u, v))
    }
}
