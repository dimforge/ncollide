//! Non-persistant pairwise geometric queries.

#[doc(inline)]
pub use self::contacts_internal::Contact;
#[doc(inline)]
pub use self::contacts_internal::contact_internal as contact;
#[doc(inline)]
pub use self::proximity_internal::Proximity;
#[doc(inline)]
pub use self::proximity_internal::proximity_internal as proximity;
#[doc(inline)]
pub use self::distance_internal::distance;
#[doc(inline)]
pub use self::time_of_impact_internal::time_of_impact;

pub mod algorithms;
pub mod contacts_internal;
pub mod distance_internal;
pub mod proximity_internal;
pub mod time_of_impact_internal;
