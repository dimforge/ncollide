use utils::data::has_uid::HasUid;
use ray::Ray;
use broad_phase::ProximitySignalHandler;

/// Filter checking if two object can be reported for proximity.
pub trait ProximityFilter<B> {
    /// Indicates if two object can be reported for proximity.
    fn is_proximity_allowed(&self, &B, &B) -> bool;
}

/// Trait all broad phase must implement.
pub trait BroadPhase<P, V, B: HasUid, BV> {
    /// Adds an element to this broad phase.
    fn add(&mut self, body: B, bv: BV);

    /// Removes an element from this broad phase.
    ///
    /// The element is identified by its unique id.
    fn remove_with_uid(&mut self, uid: uint);

    /// Sets the next bounding volume to be used during the update of this broad phase.
    ///
    /// The affected body is identified by its unique id.
    fn set_next_bounding_volume_with_uid(&mut self, uid: uint, bv: BV);

    /// Updates the interferences.
    fn update(&mut self);

    /// Registers a handler for proximity start/stop events.
    fn register_proximity_signal_handler(&mut self, name: &str, handler: Box<ProximitySignalHandler<B> + 'static>);

    /// Unregisters a handler for proximity start/stop events.
    fn unregister_proximity_signal_handler(&mut self, name: &str);

    /*
     * FIXME: the following are not flexible enough.
     */
    // XXX: return iterators when associated types work.
    /// Collects every object which might intersect a given bounding volume.
    fn interferences_with_bounding_volume<'a>(&'a self, bv: &BV, out: &mut Vec<&'a B>);

    /// Collects every object which might intersect a given ray.
    fn interferences_with_ray<'a>(&'a self, ray: &Ray<P, V>, out: &mut Vec<&'a B>);

    /// Collects every object which might contain a given point.
    fn interferences_with_point<'a>(&'a self, point: &P, out: &mut Vec<&'a B>);

    /// Removes an element from this broad phase.
    fn remove(&mut self, body: &B) {
        self.remove_with_uid(body.uid())
    }

    /// Sets the next bounding volume to be used during the update of this broad phase.
    fn set_next_bounding_volume(&mut self, body: &B, bv: BV) {
        self.set_next_bounding_volume_with_uid(body.uid(), bv)
    }
}
