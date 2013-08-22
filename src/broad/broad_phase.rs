use ray::ray::Ray;

/// Trait all broad phase must implement. This does not do much per se. Useful features are
/// provided by the traits: `InterferencesBroadPhase`, `BoundingVolumeBroadPhase`, and
/// `RayCastBroadPhase`.
pub trait BroadPhase<B> {
    /// Adds an element to this broad phase.
    fn add(&mut self, @mut B);
    /// Removes an element from this broad phase.
    fn remove(&mut self, @mut B);

    /// Updates the collision pairs based on the objects bounding volumes.
    fn update(&mut self);
    /// Updates the collision pairs involving one specific object.
    fn update_object(&mut self, @mut B);
}

/// Broad phase which check for pairwise interferences (aka. coarse collision detection).
/// This is the most common feature provided by broad phases.
pub trait InterferencesBroadPhase<B, DV> : BroadPhase<B> {
    /// Marks and object as active. Active objects are checked for interferences at each update.
    fn activate(&mut self, body: @mut B, f: &fn(@mut B, @mut B, &mut DV));
    /// Marks and object as inactive. Inactive objects are assumed to be static and not tested for
    /// mutual interferences.
    fn deactivate(&mut self, @mut B);

    /// Execute a function on each interefence detected by the broad phase.
    fn for_each_pair(&self, f: &fn(@mut B, @mut B, &DV));
    /// Execute a function on each interefence detected by the broad phase.
    fn for_each_pair_mut(&mut self, f: &fn(@mut B, @mut B, &mut DV));
}

/// Trait of broad phases working with bounding volume.
pub trait BoundingVolumeBroadPhase<B, BV> : BroadPhase<B> {
    /// Collects every object which might intersect a given bounding volume.
    fn interferences_with_bounding_volume(&mut self, &BV, &mut ~[@mut B]);
}

/// Traits of broad phase able to run fast ray-cast queries.
pub trait RayCastBroadPhase<V, B> : BroadPhase<B> {
    /// Collects every object which might intersect a ray.
    fn interferences_with_ray(&mut self, &Ray<V>, &mut ~[@mut B]);
}
