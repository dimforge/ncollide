use crate::math::Isometry;
use na::RealField;
use crate::query::{ContactManifold, ContactPrediction};
use crate::shape::Shape;
use crate::query::ContactPreprocessor;
use std::any::Any;
use crate::utils::IdAllocator;

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
        dispatcher: &ContactDispatcher<N>,
        ma: &Isometry<N>,
        a: &Shape<N>,
        proc1: Option<&ContactPreprocessor<N>>,
        mb: &Isometry<N>,
        b: &Shape<N>,
        proc2: Option<&ContactPreprocessor<N>>,
        prediction: &ContactPrediction<N>,
        id_alloc: &mut IdAllocator,
        manifold: &mut ContactManifold<N>,
    ) -> bool;

    /// Generate an empty contact manifold configured as required by this contact manifold generator.
    fn init_manifold(&self) -> ContactManifold<N> {
        ContactManifold::new()
    }
}

pub type ContactAlgorithm<N> = Box<ContactManifoldGenerator<N>>;

pub trait ContactDispatcher<N>: Any + Send + Sync {
    /// Allocate a collision algorithm corresponding to a pair of objects with the given shapes.
    fn get_contact_algorithm(&self, a: &Shape<N>, b: &Shape<N>) -> Option<ContactAlgorithm<N>>;
}
