use math::Isometry;
use na::Real;
use query::{ContactManifold, ContactPrediction};
use shape::{FeatureId, Shape};
use pipeline::narrow_phase::ContactGeneratorShapeContext;
use std::any::Any;
use utils::IdAllocator;

/// Trait implemented algorithms that compute contact points, normals and penetration depths.
pub trait ContactManifoldGenerator<N: Real>: Any + Send + Sync {
    /// Runs the collision detection on two objects. It is assumed that the same
    /// collision detector (the same structure) is always used with the same
    /// pair of object.
    fn generate_contacts(
        &mut self,
        dispatcher: &ContactDispatcher<N>,
        ma: &Isometry<N>,
        a: &Shape<N>,
        ctxt1: Option<&ContactGeneratorShapeContext<N>>,
        mb: &Isometry<N>,
        b: &Shape<N>,
        ctxt2: Option<&ContactGeneratorShapeContext<N>>,
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
    /// Allocate a collision algorithm corresponding to the given pair of shapes.
    fn get_contact_algorithm(&self, a: &Shape<N>, b: &Shape<N>) -> Option<ContactAlgorithm<N>>;
}
