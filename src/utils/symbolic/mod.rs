//! Basic symbolic computation system.

#[doc(inline)]
pub use utils::symbolic::univariate_fn::UnivariateFn;
#[doc(inline)]
pub use utils::symbolic::bivariate_fn::BivariateFn;
pub use utils::symbolic::deriv::{Deriv, deriv, DerivU, deriv_u, DerivV, deriv_v};
pub use utils::symbolic::comp::{SymComp, comp};
pub use utils::symbolic::mult::{SymMult, mult};
pub use utils::symbolic::add::{SymAdd, add};
pub use utils::symbolic::sub::{SymSub, sub};
pub use utils::symbolic::neg::{SymNeg, neg};
pub use utils::symbolic::t::{T, t};
pub use utils::symbolic::u::{U, u};
pub use utils::symbolic::v::{V, v};
pub use utils::symbolic::sin::{Sin, sin};
pub use utils::symbolic::cos::{Cos, cos};
pub use utils::symbolic::exp::{Exp, exp};
pub use utils::symbolic::poly::{Poly1, t1,
                                Poly2, t2,
                                Poly3, t3,
                                Poly4, t4,
                                Poly5, t5,
                                Poly6, t6};

mod ops;
#[doc(hidden)]
pub mod univariate_fn;
#[doc(hidden)]
pub mod bivariate_fn;
mod t;
mod u;
mod v;
mod deriv;
mod comp;
mod mult;
mod add;
mod sub;
mod neg;
mod consts;
mod sin;
mod cos;
mod exp;
mod poly;

#[cfg(test)]
mod test;
