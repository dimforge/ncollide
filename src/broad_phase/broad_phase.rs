use ray::Ray;

/// Trait all broad phase must implement.
pub trait BroadPhase<P, V, B, BV, DV> {
    /// Adds an element to this broad phase.
    fn add(&mut self, B);

    /// Removes an element from this broad phase.
    fn remove(&mut self, &B);

    /// Updates the collision pairs based on the objects bounding volumes.
    fn update(&mut self);

    /// Updates the collision pairs involving one specific object.
    fn update_object(&mut self, &B);

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

    /// Collects every object which might intersect a given bounding volume.
    fn interferences_with_bounding_volume(&mut self, &BV, &mut Vec<B>);

    /// Collects every object which might intersect a given ray.
    fn interferences_with_ray(&mut self, &Ray<P, V>, &mut Vec<B>);

    /// Collects every object which might contain a given point.
    fn interferences_with_point(&mut self, &P, &mut Vec<B>);
}
