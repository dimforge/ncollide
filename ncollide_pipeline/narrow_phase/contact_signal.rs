use world::CollisionObject;
use narrow_phase::ContactAlgorithm;
use math::Point;

/// A signal handler for contact detection.
pub trait ContactHandler<P: Point, M, T> : Sync + Send {
    /// Activate an action for when two objects start being in contact.
    fn handle_contact_started(&mut self,
                              co1:      &CollisionObject<P, M, T>,
                              co2:      &CollisionObject<P, M, T>,
                              contacts: &ContactAlgorithm<P, M>);

    /// Activate an action for when two objects stop being in contact.
    fn handle_contact_stopped(&mut self, co1: &CollisionObject<P, M, T>, co2: &CollisionObject<P, M, T>);
}

/// Signal for contact start/stop.
pub struct ContactSignal<P: Point, M, T> {
    contact_handlers: Vec<(String, Box<ContactHandler<P, M, T> + 'static>)>,
}

impl<P: Point, M, T> ContactSignal<P, M, T> {
    /// Creates a new `ContactSignal` with no event handler registered.
    pub fn new() -> ContactSignal<P, M, T> {
        ContactSignal {
            contact_handlers: Vec::new(),
        }
    }

    /// Registers an event handler.
    pub fn register_contact_handler(&mut self,
                                    name:     &str,
                                    callback: Box<ContactHandler<P, M, T> + 'static>) {
        for &mut (ref mut n, ref mut f) in self.contact_handlers.iter_mut() {
            if name == &n[..] {
                *f = callback;
                return;
            }
        }

        self.contact_handlers.push((name.to_string(), callback))
    }

    /// Unregisters an event handler.
    pub fn unregister_contact_handler(&mut self, name: &str) {
        let mut to_remove = self.contact_handlers.len();

        for (i, &mut (ref n, _)) in self.contact_handlers.iter_mut().enumerate() {
            if name == &n[..] {
                to_remove = i;
            }
        }

        if to_remove != self.contact_handlers.len() {
            let _ = self.contact_handlers.remove(to_remove);
        }
    }

    // FIXME: do we really want to use &mut here ?
    /// Activates the contact started signal, executing all the event handlers.
    pub fn trigger_contact_started_signal(&mut self,
                                          co1:      &CollisionObject<P, M, T>,
                                          co2:      &CollisionObject<P, M, T>,
                                          contacts: &ContactAlgorithm<P, M>) {
        for &mut (_, ref mut f) in self.contact_handlers.iter_mut() {
            f.handle_contact_started(co1, co2, contacts)
        }
    }

    /// Activates the contact stopped signal, executing all the event handlers.
    pub fn trigger_contact_stopped_signal(&mut self,
                                          co1: &CollisionObject<P, M, T>,
                                          co2: &CollisionObject<P, M, T>) {
        for &mut (_, ref mut f) in self.contact_handlers.iter_mut() {
            f.handle_contact_stopped(co1, co2)
        }
    }
}
