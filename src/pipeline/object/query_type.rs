use crate::query::ContactPrediction;
use na::RealField;

/// The kind of query a CollisionObject may be involved on.
///
/// The following queries are executed for a given pair of `GeometricQueryType` associated with two
/// collision objects:
///
/// * Contacts + Contacts = exact contact point coputation.
/// * Contacts + Proximity = proximity test only.
/// * Proximity + Proximity = proximity test only.
#[derive(Debug, PartialEq, Clone, Copy)]
pub enum GeometricQueryType<N: RealField> {
    /// This objects can respond to both contact point computation and proximity queries.
    Contacts(N, N),
    /// This object can respond to proximity tests only.
    Proximity(N),
    // FIXME: not yet implemented: Distance
}

impl<N: RealField> GeometricQueryType<N> {
    /// The numerical distance limit of relevance for this query.
    ///
    /// If two objects are separated by a distance greater than the sum of their respective
    /// `query_limit`, the corresponding query will not by performed. For proximity queries,
    /// non-intersecting object closer than a distance equal to the sum of their `query_limit` will
    /// be reported as `Proximity::WithinMargin`.
    #[inline]
    pub fn query_limit(&self) -> N {
        match *self {
            GeometricQueryType::Contacts(ref val, _) => *val,
            GeometricQueryType::Proximity(ref val) => *val,
        }
    }

    /// Given two contact query types, returns the corresponding contact prediction parameters.
    ///
    /// Returns `None` if any of `self` or `other` is not a `GeometricQueryType::Contacts`.
    pub fn contact_queries_to_prediction(self, other: Self) -> Option<ContactPrediction<N>> {
        match (self, other) {
            (
                GeometricQueryType::Contacts(linear1, angular1),
                GeometricQueryType::Contacts(linear2, angular2),
            ) => Some(ContactPrediction::new(
                linear1 + linear2,
                angular1,
                angular2,
            )),
            _ => None,
        }
    }

    /// Returns `true` if this is a contacts query type.
    #[inline]
    pub fn is_contacts_query(&self) -> bool {
        if let GeometricQueryType::Contacts(..) = *self {
            true
        } else {
            false
        }
    }

    /// Returns `true` if this is a proximity query type.
    #[inline]
    pub fn is_proximity_query(&self) -> bool {
        if let GeometricQueryType::Proximity(_) = *self {
            true
        } else {
            false
        }
    }
}
