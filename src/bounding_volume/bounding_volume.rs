/// Traits of objects having a bounding volume.
pub trait HasBoundingVolume<BV> {
    /// The object bounding volume.
    fn bounding_volume(&self) -> BV;
}

/// Trait of bounding volumes.
///
/// Bounding volumes are coarse approximations of geometric primitives.  It usually have constant
/// time intersection, inclusion test. Two bounding volume must also be mergeable into a bigger
/// bounding volume.
pub trait BoundingVolume<N> {
    /// Checks if this bounding volume intersect with another one.
    fn intersects(&self, &Self) -> bool;

    /// Checks if this bounding volume contains another one.
    fn contains(&self, &Self)   -> bool;

    /// Merges this bounding volume with another one. The merge is done in-place.
    fn merge(&mut self, &Self);

    /// Merges this bounding volume with another one.
    fn merged(&self, &Self) -> Self;

    /// Enlarges this bounding volume.
    fn loosen(&mut self, N);

    /// Creates a new, enlarged version, of this bounding volume.
    fn loosened(&self, N) -> Self;

    /// Tighten this bounding volume.
    fn tighten(&mut self, N);

    /// Creates a new, tightened version, of this bounding volume.
    fn tightened(&self, N) -> Self;
}
