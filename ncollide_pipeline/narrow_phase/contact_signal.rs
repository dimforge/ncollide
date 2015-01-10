/// A signal handler for contact detection.
pub trait ContactSignalHandler<B> {
    /// Activate an action for when two objects start or stop to be close to each other.
    fn handle_contact(&mut self, b1: &B, b2: &B, started: bool);
}

/// Signal for contact start/stop.
pub struct ContactSignal<B> {
    contact_signal_handlers: Vec<(String, Box<ContactSignalHandler<B> + 'static>)>,
}

impl<B> ContactSignal<B> {
    /// Creates a new `ContactSignal` with no event handler registered.
    pub fn new() -> ContactSignal<B> {
        ContactSignal {
            contact_signal_handlers: Vec::new(),
        }
    }

    /// Registers an event handler.
    pub fn register_contact_signal_handler(&mut self,
                                             name:     &str,
                                             callback: Box<ContactSignalHandler<B> + 'static>) {
        for &mut (ref mut n, ref mut f) in self.contact_signal_handlers.iter_mut() {
            if name == n.as_slice() {
                *f = callback;
                return;
            }
        }

        self.contact_signal_handlers.push((name.to_string(), callback))
    }

    /// Unregisters an event handler.
    pub fn unregister_contact_signal_handler(&mut self, name: &str) {
        let mut to_remove = self.contact_signal_handlers.len();

        for (i, &mut (ref n, _)) in self.contact_signal_handlers.iter_mut().enumerate() {
            if name == n.as_slice() {
                to_remove = i;
            }
        }

        if to_remove != self.contact_signal_handlers.len() {
            let _ = self.contact_signal_handlers.remove(to_remove);
        }
    }

    // FIXME: do we really want to use &mut here ?
    /// Activates the contact signal, executing all the event handlers.
    pub fn trigger_contact_signal(&mut self, b1: &B, b2: &B, started: bool) {
        for &mut (_, ref mut f) in self.contact_signal_handlers.iter_mut() {
            f.handle_contact(b1, b2, started)
        }
    }
}
