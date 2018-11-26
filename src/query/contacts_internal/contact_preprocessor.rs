use na::Real;
use math::Isometry;
use shape::FeatureId;
use utils::IsometryOps;
use query::{Contact, ContactPrediction, ContactKinematic};


/// Pre-process a contact before it is added to a contact manifold.
pub trait ContactPreprocessor<N: Real> {
    fn process_contact(
        &self,
        c: &mut Contact<N>,
        kinematic: &mut ContactKinematic<N>,
        is_first: bool)
        -> bool;
}