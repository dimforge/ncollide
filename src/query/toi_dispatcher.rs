use na::RealField;

use crate::interpolation::RigidMotion;
use crate::math::{Isometry, Vector};
use crate::query::{self, Unsupported, TOI};
use crate::shape::Shape;

/// Dispatcher for time-of-impact queries
///
/// Custom implementations allow crates that support an abstract `TOIDispatcher` to handle custom
/// shapes. Methods take `root_dispatcher` to allow dispatchers to delegate to eachother. Callers
/// that will not themselves be used to implement a `TOIDispatcher` should pass `self`.
pub trait TOIDispatcher<N: RealField>: Send + Sync {
    /// Computes the smallest time of impact of two shapes under translational movement.
    fn nonlinear_time_of_impact(
        &self,
        root_dispatcher: &dyn TOIDispatcher<N>,
        motion1: &dyn RigidMotion<N>,
        g1: &dyn Shape<N>,
        motion2: &dyn RigidMotion<N>,
        g2: &dyn Shape<N>,
        max_toi: N,
        target_distance: N,
    ) -> Result<Option<TOI<N>>, Unsupported>;

    /// Computes the smallest time at with two shapes under translational movement are separated by a
    /// distance smaller or equal to `distance`.
    ///
    /// Returns `0.0` if the objects are touching or penetrating.
    fn time_of_impact(
        &self,
        root_dispatcher: &dyn TOIDispatcher<N>,
        m1: &Isometry<N>,
        vel1: &Vector<N>,
        g1: &dyn Shape<N>,
        m2: &Isometry<N>,
        vel2: &Vector<N>,
        g2: &dyn Shape<N>,
        max_toi: N,
        target_distance: N,
    ) -> Result<Option<TOI<N>>, Unsupported>;

    /// Construct a `TOIDispatcher` that falls back on `other` for cases not handled by `self`
    fn chain<U: TOIDispatcher<N>>(self, other: U) -> Chain<Self, U>
    where
        Self: Sized,
    {
        Chain(self, other)
    }
}

/// A dispatcher that exposes built-in queries
#[derive(Debug, Clone)]
pub struct DefaultTOIDispatcher;

impl<N: RealField> TOIDispatcher<N> for DefaultTOIDispatcher {
    fn nonlinear_time_of_impact(
        &self,
        root_dispatcher: &dyn TOIDispatcher<N>,
        motion1: &dyn RigidMotion<N>,
        g1: &dyn Shape<N>,
        motion2: &dyn RigidMotion<N>,
        g2: &dyn Shape<N>,
        max_toi: N,
        target_distance: N,
    ) -> Result<Option<TOI<N>>, Unsupported> {
        query::nonlinear_time_of_impact(
            root_dispatcher,
            motion1,
            g1,
            motion2,
            g2,
            max_toi,
            target_distance,
        )
    }

    fn time_of_impact(
        &self,
        root_dispatcher: &dyn TOIDispatcher<N>,
        m1: &Isometry<N>,
        vel1: &Vector<N>,
        g1: &dyn Shape<N>,
        m2: &Isometry<N>,
        vel2: &Vector<N>,
        g2: &dyn Shape<N>,
        max_toi: N,
        target_distance: N,
    ) -> Result<Option<TOI<N>>, Unsupported> {
        query::time_of_impact(
            root_dispatcher,
            m1,
            vel1,
            g1,
            m2,
            vel2,
            g2,
            max_toi,
            target_distance,
        )
    }
}

/// The composition of two dispatchers
pub struct Chain<T, U>(T, U);

macro_rules! chain_method {
    ($name:ident ( $( $arg:ident : $ty:ty,)*) -> $result:ty) => {
        fn $name(&self, root_dispatcher: &dyn TOIDispatcher<N>,
                 $($arg : $ty,)*
        ) -> Result<$result, Unsupported> {
            (self.0).$name(root_dispatcher, $($arg,)*)
                .or_else(|Unsupported| (self.1).$name(root_dispatcher, $($arg,)*))
        }
    }
}

impl<N, T, U> TOIDispatcher<N> for Chain<T, U>
where
    N: RealField,
    T: TOIDispatcher<N>,
    U: TOIDispatcher<N>,
{
    chain_method!(nonlinear_time_of_impact(
        motion1: &dyn RigidMotion<N>,
        g1: &dyn Shape<N>,
        motion2: &dyn RigidMotion<N>,
        g2: &dyn Shape<N>,
        max_toi: N,
        target_distance: N,
    ) -> Option<TOI<N>>);

    chain_method!(time_of_impact(
        m1: &Isometry<N>,
        vel1: &Vector<N>,
        g1: &dyn Shape<N>,
        m2: &Isometry<N>,
        vel2: &Vector<N>,
        g2: &dyn Shape<N>,
        max_toi: N,
        target_distance: N,
    ) -> Option<TOI<N>>);
}
