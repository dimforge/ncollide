//! Non-persistant geometric queries.

#[doc(inline)]
pub use self::contacts_internal::Contact;
#[doc(inline)]
pub use self::contacts_internal::contacts_with::{contact, contacts};
#[doc(inline)]
pub use self::distance_internal::distance_with::distance;
#[doc(inline)]
pub use self::time_of_impact_internal::time_of_impact_with::time_of_impact;

pub mod algorithms;
// pub mod closest_points;
pub mod contacts_internal;
pub mod distance_internal;
pub mod time_of_impact_internal;
