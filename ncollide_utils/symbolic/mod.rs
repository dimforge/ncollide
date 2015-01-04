//! Basic symbolic computation system.

#[doc(inline)]
pub use symbolic::univariate_fn::UnivariateFn;
#[doc(inline)]
pub use symbolic::bivariate_fn::BivariateFn;
pub use symbolic::deriv::{Deriv, deriv, DerivU, deriv_u, DerivV, deriv_v};
pub use symbolic::comp::{SymComp, comp};
pub use symbolic::mult::{SymMult, mult};
pub use symbolic::add::{SymAdd, add};
pub use symbolic::sub::{SymSub, sub};
pub use symbolic::neg::{SymNeg, neg};
pub use symbolic::t::{T, t};
pub use symbolic::u::{U, u};
pub use symbolic::v::{V, v};
pub use symbolic::sin::{Sin, sin};
pub use symbolic::cos::{Cos, cos};
pub use symbolic::exp::{Exp, exp};
pub use symbolic::poly::{Poly1, t1,
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
