//! Structures for describing and storing collision-related events.

use crate::pipeline::world::CollisionObjectHandle;
use crate::query::Proximity;
use std::iter::IntoIterator;
use std::slice::Iter;

// FIXME: we want a structure where we can add elements, iterate on them, but not remove them
// without clearing the whole structure.
/// A set of events.
pub struct EventPool<E> {
    events: Vec<E>,
}

/// A set of contact events.
pub type ContactEvents = EventPool<ContactEvent>;
/// A set of proximity events.
pub type ProximityEvents = EventPool<ProximityEvent>;
/// A set of contact events.
pub type ContactEvents2<Handle> = EventPool<ContactEvent2<Handle>>;
/// A set of proximity events.
pub type ProximityEvents2<Handle> = EventPool<ProximityEvent2<Handle>>;

impl<E> EventPool<E> {
    /// Creates a new empty set of events.
    pub fn new() -> EventPool<E> {
        EventPool { events: Vec::new() }
    }

    /// Empties this set of events.
    pub fn clear(&mut self) {
        self.events.clear();
    }

    /// Adds the given event at the end of this set.
    pub fn push(&mut self, event: E) {
        self.events.push(event);
    }

    /// Iterates through all events contained on this set in a FIFO manner.
    pub fn iter(&self) -> Iter<E> {
        self.events.iter()
    }

    /// Removes from this set all events for which `filter` returns `false`.
    pub fn retain<F>(&mut self, filter: F)
    where F: FnMut(&E) -> bool {
        self.events.retain(filter)
    }

    /// The number of events on this pool.
    #[inline]
    pub fn len(&self) -> usize {
        self.events.len()
    }
}

impl<'a, E> IntoIterator for &'a EventPool<E> {
    type Item = &'a E;
    type IntoIter = Iter<'a, E>;

    fn into_iter(self) -> Iter<'a, E> {
        (&self.events).into_iter()
    }
}

#[derive(Copy, Clone, Hash, Debug)]
/// Events occuring when two collision objects start or stop being in contact (or penetration).
pub enum ContactEvent {
    /// Event occuring when two collision objects start being in contact.
    ///
    /// This event is generated whenever the narrow-phase finds a contact between two collision objects that did not have any contact at the last update.
    Started(CollisionObjectHandle, CollisionObjectHandle),
    /// Event occuring when two collision objects stop being in contact.    
    /// 
    /// This event is generated whenever the narrow-phase fails to find any contact between two collision objects that did have at least one contact at the last update.
    Stopped(CollisionObjectHandle, CollisionObjectHandle),
}

#[derive(Copy, Clone, Debug)]
/// Events occuring when two collision objects start or stop being in close proximity, contact, or disjoint.
pub struct ProximityEvent {
    /// The first collider to which the proximity event applies.
    pub collider1: CollisionObjectHandle,
    /// The second collider to which the proximity event applies.
    pub collider2: CollisionObjectHandle,
    /// The previous state of proximity between the two collision objects.
    pub prev_status: Proximity,
    /// The new state of proximity between the two collision objects.
    pub new_status: Proximity,
}

impl ProximityEvent {
    /// Instaciates a new proximity event.
    ///
    /// Panics if `prev_status` is equal to `new_status`.
    pub fn new(
        collider1: CollisionObjectHandle,
        collider2: CollisionObjectHandle,
        prev_status: Proximity,
        new_status: Proximity,
    ) -> ProximityEvent
    {
        assert!(prev_status != new_status);
        ProximityEvent {
            collider1,
            collider2,
            prev_status,
            new_status,
        }
    }
}


#[derive(Copy, Clone, Hash, Debug)]
/// Events occuring when two collision objects start or stop being in contact (or penetration).
pub enum ContactEvent2<Handle> {
    /// Event occuring when two collision objects start being in contact.
    ///
    /// This event is generated whenever the narrow-phase finds a contact between two collision objects that did not have any contact at the last update.
    Started(Handle, Handle),
    /// Event occuring when two collision objects stop being in contact.
    ///
    /// This event is generated whenever the narrow-phase fails to find any contact between two collision objects that did have at least one contact at the last update.
    Stopped(Handle, Handle),
}

#[derive(Copy, Clone, Debug)]
/// Events occuring when two collision objects start or stop being in close proximity, contact, or disjoint.
pub struct ProximityEvent2<Handle> {
    /// The first collider to which the proximity event applies.
    pub collider1: Handle,
    /// The second collider to which the proximity event applies.
    pub collider2: Handle,
    /// The previous state of proximity between the two collision objects.
    pub prev_status: Proximity,
    /// The new state of proximity between the two collision objects.
    pub new_status: Proximity,
}

impl<Handle> ProximityEvent2<Handle> {
    /// Instantiates a new proximity event.
    ///
    /// Panics if `prev_status` is equal to `new_status`.
    pub fn new(
        collider1: Handle,
        collider2: Handle,
        prev_status: Proximity,
        new_status: Proximity,
    ) -> Self
    {
        assert_ne!(prev_status, new_status, "The previous and new status of a proximity event must not be the same.");
        Self {
            collider1,
            collider2,
            prev_status,
            new_status,
        }
    }
}
