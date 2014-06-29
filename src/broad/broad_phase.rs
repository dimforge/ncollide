use ray::Ray;

/// Trait all broad phase must implement.
///
/// This does not do much per se. Useful features are provided by the
/// traits: `InterferencesBroadPhase`, `BoundingVolumeBroadPhase`, and `RayCastBroadPhase`.
pub trait BroadPhase<B> {
    /// Adds an element to this broad phase.
    fn add(&mut self, B);
    /// Removes an element from this broad phase.
    fn remove(&mut self, &B);

    /// Updates the collision pairs based on the objects bounding volumes.
    fn update(&mut self);
    /// Updates the collision pairs involving one specific object.
    fn update_object(&mut self, &B);
}

/// Thait of broad phases which check for pairwise interferences.
///
/// This is the most common feature provided by broad phases.
pub trait InterferencesBroadPhase<B, DV> : BroadPhase<B> {
    /// Marks and object as active.
    ///
    /// Active objects are checked for interferences at each update.
    fn activate(&mut self, body: &B, f: |&B, &B, &mut DV| -> ());
    /// Marks and object as inactive.
    ///
    /// Inactive objects are assumed to be static and not tested for mutual interferences.
    fn deactivate(&mut self, &B);

    /// Execute a function on each interference detected by the broad phase.
    fn for_each_pair(&self, f: |&B, &B, &DV| -> ());
    /// Execute a function on each interference detected by the broad phase.
    fn for_each_pair_mut(&mut self, f: |&B, &B, &mut DV| -> ());
}

/// Trait of broad phases working with bounding volume.
pub trait BoundingVolumeBroadPhase<B, BV> : BroadPhase<B> {
    /// Collects every object which might intersect a given bounding volume.
    fn interferences_with_bounding_volume(&mut self, &BV, &mut Vec<B>);
}

/// Traits of broad phase able to run fast ray-cast queries.
pub trait RayCastBroadPhase<B> : BroadPhase<B> {
    /// Collects every object which might intersect a ray.
    fn interferences_with_ray(&mut self, &Ray, &mut Vec<B>);
}
