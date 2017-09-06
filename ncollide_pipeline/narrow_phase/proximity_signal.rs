use geometry::query::Proximity;
use world::CollisionObject;
use math::Point;

/// A signal handler for proximity detection.
pub trait ProximityHandler<P: Point, M, T> : Sync + Send {
    /// Activate an action for when two objects start or stop to be close to each other.
    fn handle_proximity(&mut self,
                        co1: &CollisionObject<P, M, T>,
                        co2: &CollisionObject<P, M, T>,
                        prev_status: Proximity,
                        new_status:  Proximity);
}

/// Signal for proximity start/stop.
pub struct ProximitySignal<P: Point, M, T> {
    proximity_handlers: Vec<(String, Box<ProximityHandler<P, M, T> + 'static>)>,
}

impl<P: Point, M, T> ProximitySignal<P, M, T> {
    /// Creates a new `ProximitySignal` with no event handler registered.
    pub fn new() -> ProximitySignal<P, M, T> {
        ProximitySignal {
            proximity_handlers: Vec::new(),
        }
    }

    /// Registers an event handler.
    pub fn register_proximity_handler(&mut self,
                                      name:     &str,
                                      callback: Box<ProximityHandler<P, M, T> + 'static>) {
        for &mut (ref mut n, ref mut f) in self.proximity_handlers.iter_mut() {
            if name == &n[..] {
                *f = callback;
                return;
            }
        }

        self.proximity_handlers.push((name.to_string(), callback))
    }

    /// Unregisters an event handler.
    pub fn unregister_proximity_handler(&mut self, name: &str) {
        let mut to_remove = self.proximity_handlers.len();

        for (i, &mut (ref n, _)) in self.proximity_handlers.iter_mut().enumerate() {
            if name == &n[..] {
                to_remove = i;
            }
        }

        if to_remove != self.proximity_handlers.len() {
            let _ = self.proximity_handlers.remove(to_remove);
        }
    }

    // FIXME: do we really want to use &mut here ?
    /// Activates the proximity signal, executing all the event handlers.
    pub fn trigger_proximity_signal(&mut self,
                                    co1: &CollisionObject<P, M, T>,
                                    co2: &CollisionObject<P, M, T>,
                                    prev_status: Proximity,
                                    new_status: Proximity) {
        for &mut (_, ref mut f) in self.proximity_handlers.iter_mut() {
            f.handle_proximity(co1, co2, prev_status, new_status)
        }
    }
}
