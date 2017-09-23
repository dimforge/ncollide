use std::slice::Iter;
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
}

#[derive(Copy, Clone, Hash, Debug)]
pub enum ContactEvent {
    Started(CollisionObjectHandle, CollisionObjectHandle),
    Stopped(CollisionObjectHandle, CollisionObjectHandle)
}

#[derive(Copy, Clone, Debug)]
pub struct ProximityEvent {
    co1:         CollisionObjectHandle,
    co2:         CollisionObjectHandle,
    prev_status: Proximity,
    new_status:  Proximity
}

impl ProximityEvent {
    pub fn new(co1:         CollisionObjectHandle,
               co2:         CollisionObjectHandle,
               prev_status: Proximity,
               new_status:  Proximity)
              -> ProximityEvent {
        ProximityEvent {
            co1, co2, prev_status, new_status
        }
    }
}
