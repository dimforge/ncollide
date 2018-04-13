//! Non-persistant pairwise geometric queries.

#[doc(inline)]
pub use self::contacts_internal::{Contact, ContactKinematic, ContactManifold, ContactPrediction,
                                  TrackedContact};
// #[doc(inline)]
// pub use self::contacts_internal::contact_internal as contact;
#[doc(inline)]
pub use self::closest_points_internal::ClosestPoints;
// #[doc(inline)]
// pub use self::closest_points_internal::closest_points_internal as closest_points;
// #[doc(inline)]
// pub use self::proximity_internal::Proximity;
// #[doc(inline)]
// pub use self::proximity_internal::proximity_internal as proximity;
// #[doc(inline)]
// pub use self::distance_internal::distance;
// #[doc(inline)]
// pub use self::time_of_impact_internal::time_of_impact;
// #[doc(inline)]
// pub use self::ray_internal::{Ray, Ray2, Ray3, RayCast, RayInterferencesCollector, RayIntersection,
//                              RayIntersection2, RayIntersection3, RayIntersectionCostFn};
#[doc(inline)]
pub use self::point_internal::{PointInterferencesCollector, PointProjection, PointQuery,
                               PointQueryWithLocation};

// pub mod algorithms;
pub mod contacts_internal;
pub mod closest_points_internal;
// pub mod distance_internal;
// pub mod proximity_internal;
// pub mod time_of_impact_internal;
// pub mod ray_internal;
pub mod point_internal;
