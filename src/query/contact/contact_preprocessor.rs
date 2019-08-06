use na::RealField;
use crate::query::{Contact, ContactKinematic};


/// Pre-process a contact before it is added to a contact manifold.
pub trait ContactPreprocessor<N: RealField> {
    /// Process a contact before it is stored into a contact manifold.
    ///
    /// Returns `false` if the contact should be ignored.
    fn process_contact(
        &self,
        c: &mut Contact<N>,
        kinematic: &mut ContactKinematic<N>,
        is_first: bool)
        -> bool;
}

// FIXME: not sure if there is a more efficient way of doing this.
// In particular, this induces some overhead (additional indirection and possibly one
// more virtual call) when the first member is `None`.
impl<'a, 'b, N, A, B> ContactPreprocessor<N> for (Option<&'a A>, &'b B)
    where N: RealField,
          A: ?Sized + ContactPreprocessor<N>,
          B: ?Sized + ContactPreprocessor<N> {
    fn process_contact(
        &self,
        c: &mut Contact<N>,
        kinematic: &mut ContactKinematic<N>,
        is_first: bool)
        -> bool {
        if let Some(p) = self.0 {
            p.process_contact(c, kinematic, is_first) &&
                self.1.process_contact(c, kinematic, is_first)
        } else {
            self.1.process_contact(c, kinematic, is_first)
        }
    }
}