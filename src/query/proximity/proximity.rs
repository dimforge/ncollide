/// Proximity information.
#[derive(Debug, PartialEq, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum Proximity {
    /// The two objects are intersecting.
    Intersecting,
    /// The two objects are non-intersecting but closer than a given distance.
    WithinMargin,
    /// The two objects are non-intersecting and further than a given distance.
    Disjoint,
}