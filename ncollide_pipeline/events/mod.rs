use std::slice::Iter;
use std::iter::IntoIterator;
use geometry::query::Proximity;
use world::CollisionObjectHandle;

// FIXME: we want a structure where we can add elements, iterate on them, but not remove them
// without clearing the whole structure.
pub struct EventPool<E> {
    events: Vec<E>
}

pub type ContactEvents   = EventPool<ContactEvent>;
pub type ProximityEvents = EventPool<ProximityEvent>;


impl<E> EventPool<E> {
    pub fn new() -> EventPool<E> {
        EventPool {
            events: Vec::new()
        }
    }

    pub fn clear(&mut self) {
        self.events.clear();
    }

    pub fn push(&mut self, event: E) {
        self.events.push(event);
    }

    pub fn iter(&self) -> Iter<E> {
        self.events.iter()
    }

    pub fn retain<F>(&mut self, filter: F)
        where F: FnMut(&E) -> bool {
        self.events.retain(filter)
    }
}

impl<'a, E> IntoIterator for &'a EventPool<E> {
    type Item     = &'a E;
    type IntoIter = Iter<'a, E>;

    fn into_iter(self) -> Iter<'a, E> {
        (&self.events).into_iter()
    }
}

#[derive(Copy, Clone, Hash, Debug)]
pub enum ContactEvent {
    Started(CollisionObjectHandle, CollisionObjectHandle),
    Stopped(CollisionObjectHandle, CollisionObjectHandle)
}

#[derive(Copy, Clone, Debug)]
pub struct ProximityEvent {
    pub collider1:   CollisionObjectHandle,
    pub collider2:   CollisionObjectHandle,
    pub prev_status: Proximity,
    pub new_status:  Proximity
}

impl ProximityEvent {
    pub fn new(collider1:   CollisionObjectHandle,
               collider2:   CollisionObjectHandle,
               prev_status: Proximity,
               new_status:  Proximity)
              -> ProximityEvent {
        ProximityEvent {
            collider1, collider2, prev_status, new_status
        }
    }
}
