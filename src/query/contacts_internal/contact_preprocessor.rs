use na::Real;
use crate::query::{Contact, ContactKinematic};


/// Pre-process a contact before it is added to a contact manifold.
pub trait ContactPreprocessor<N: Real> {
    /// Process a contact before it is stored into a contact manifold.
    fn process_contact(
        &self,
        c: &mut Contact<N>,
        kinematic: &mut ContactKinematic<N>,
        is_first: bool)
        -> bool;
}