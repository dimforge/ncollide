
/// A signal handler for proximity detection.
pub trait ProximitySignalHandler<B> {
    /// Activate an action for when two objects start or stop to be close to each other.
    fn handle_proximity(&mut self, b1: &B, b2: &B, started: bool);
}

/// Signal for proximity start/stop.
pub struct ProximitySignal<B> {
    proximity_signal_handlers: Vec<(String, Box<ProximitySignalHandler<B> + 'static>)>,
}

impl<B> ProximitySignal<B> {
    /// Creates a new `ProximitySignal` with no event handler registered.
    pub fn new() -> ProximitySignal<B> {
        ProximitySignal {
            proximity_signal_handlers: Vec::new(),
        }
    }

    /// Registers an event handler.
    pub fn register_proximity_signal_handler(&mut self,
                                             name:     &str,
                                             callback: Box<ProximitySignalHandler<B> + 'static>) {
        for &(ref mut n, ref mut f) in self.proximity_signal_handlers.iter_mut() {
            if name == n.as_slice() {
                *f = callback;
                return;
            }
        }

        self.proximity_signal_handlers.push((name.to_string(), callback))
    }

    /// Unregisters an event handler.
    pub fn unregister_proximity_signal_handler(&mut self, name: &str) {
        let mut to_remove = self.proximity_signal_handlers.len();

        for (i, &(ref n, _)) in self.proximity_signal_handlers.iter_mut().enumerate() {
            if name == n.as_slice() {
                to_remove = i;
            }
        }

        if to_remove != self.proximity_signal_handlers.len() {
            let _ = self.proximity_signal_handlers.remove(to_remove);
        }
    }

    // FIXME: do we really want to use &mut here ?
    /// Activates the proximity signal, executing all the event handlers.
    pub fn trigger_proximity_signal(&mut self, b1: &B, b2: &B, started: bool) {
        for &(_, ref mut f) in self.proximity_signal_handlers.iter_mut() {
            f.handle_proximity(b1, b2, started)
        }
    }
}
