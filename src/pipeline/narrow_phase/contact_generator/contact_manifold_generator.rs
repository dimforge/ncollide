use crate::math::Isometry;
use crate::query::ContactPreprocessor;
use crate::query::{ContactManifold, ContactPrediction};
use crate::shape::Shape;
use na::RealField;
use std::any::Any;

/// An algorithm to compute contact points, normals and penetration depths between two specific
/// objects.
pub trait ContactManifoldGenerator<N: RealField>: Any + Send + Sync {
    /// Runs the collision detection on two objects. It is assumed that the same
    /// collision detector (the same structure) is always used with the same
    /// pair of objects.
    ///
    /// Returns `false` if persisting this algorithm for re-use is unlikely to improve performance,
    /// e.g. due to the objects being distant. Note that if the `ContactManifoldGenerator` would
    /// likely be immediately reconstructed in the next time-step, dropping it is sub-optimal
    /// regardless.
    fn generate_contacts(
        &mut self,
        dispatcher: &dyn ContactDispatcher<N>,
        ma: &Isometry<N>,
        a: &dyn Shape<N>,
        proc1: Option<&dyn ContactPreprocessor<N>>,
        mb: &Isometry<N>,
        b: &dyn Shape<N>,
        proc2: Option<&dyn ContactPreprocessor<N>>,
        prediction: &ContactPrediction<N>,
        manifold: &mut ContactManifold<N>,
    ) -> bool;

    /// Generate an empty contact manifold configured as required by this contact manifold generator.
    fn init_manifold(&self) -> ContactManifold<N> {
        ContactManifold::new()
    }

    /// Construct a version of `Self` that flips the arguments corresponding to each object.
    ///
    /// Allows implementations to assume a particular argument order (e.g. placing a shape of a
    /// particular type first) for convenience, while still allowing a `ContactDispatcher` to
    /// support either argument ordering.
    fn flip(self) -> FlippedContactManifoldGenerator<Self>
    where
        Self: Sized,
    {
        FlippedContactManifoldGenerator(self)
    }
}

pub struct FlippedContactManifoldGenerator<T>(T);

impl<N: RealField, T: ContactManifoldGenerator<N>> ContactManifoldGenerator<N>
    for FlippedContactManifoldGenerator<T>
{
    fn generate_contacts(
        &mut self,
        d: &dyn ContactDispatcher<N>,
        ma: &Isometry<N>,
        a: &dyn Shape<N>,
        proc1: Option<&dyn ContactPreprocessor<N>>,
        mb: &Isometry<N>,
        b: &dyn Shape<N>,
        proc2: Option<&dyn ContactPreprocessor<N>>,
        prediction: &ContactPrediction<N>,
        manifold: &mut ContactManifold<N>,
    ) -> bool {
        let dispatcher = FlippedContactDispatcher(d);
        manifold.flip_new_contacts();
        let result = self.0.generate_contacts(
            &dispatcher,
            ma,
            a,
            proc1,
            mb,
            b,
            proc2,
            prediction,
            manifold,
        );
        manifold.flip_new_contacts();
        result
    }
}

pub type ContactAlgorithm<N> = Box<dyn ContactManifoldGenerator<N>>;

pub trait ContactDispatcher<N>: Send + Sync {
    /// Allocate a collision algorithm corresponding to a pair of objects with the given shapes.
    ///
    /// Shorthand for `self.get_flipped_contact_algorithm(false, a, b)`.
    fn get_contact_algorithm(
        &self,
        a: &dyn Shape<N>,
        b: &dyn Shape<N>,
    ) -> Option<ContactAlgorithm<N>> {
        self.get_flipped_contact_algorithm(false, a, b)
    }

    /// Allocate a collision algorithm corresponding to a pair of objects with the given shapes.
    ///
    /// If `flip` is true, `a` and `b` are reversed, as are the corresponding arguments to any
    /// returned `ContactAlgorithm`.
    fn get_flipped_contact_algorithm(
        &self,
        flip: bool,
        a: &dyn Shape<N>,
        b: &dyn Shape<N>,
    ) -> Option<ContactAlgorithm<N>>;
}

/// A view of a `ContactDispatcher` which inverts the value of `flip` in `get_flipped_contact_algorithm`
struct FlippedContactDispatcher<'a, N>(&'a dyn ContactDispatcher<N>);

impl<N> ContactDispatcher<N> for FlippedContactDispatcher<'_, N> {
    fn get_flipped_contact_algorithm(
        &self,
        flip: bool,
        a: &dyn Shape<N>,
        b: &dyn Shape<N>,
    ) -> Option<ContactAlgorithm<N>> {
        self.0.get_flipped_contact_algorithm(!flip, a, b)
    }
}
