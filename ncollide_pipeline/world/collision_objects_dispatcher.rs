use utils::data::hash_map::HashMap;
use utils::data::pair::{Pair, PairTWHash};
use utils::data::uid_remap::{UidRemap, FastKey};
use queries::geometry::Contact;
use narrow_phase::{CollisionDispatcher, CollisionAlgorithm, ContactSignal, ContactSignalHandler};
use world::CollisionObject;
use math::Point;

// FIXME: move this to the `narrow_phase` module.
/// Collision detector dispatcher for collision objects.
pub struct CollisionObjectsDispatcher<P, M, T> {
    signal:           ContactSignal<T>,
    shape_dispatcher: Box<CollisionDispatcher<P, M> + 'static>,
    pairs:            HashMap<Pair, CollisionAlgorithm<P, M>, PairTWHash>
}

impl<P: Point, M, T> CollisionObjectsDispatcher<P, M, T> {
    /// Creates a new `CollisionObjectsDispatcher`.
    pub fn new(shape_dispatcher: Box<CollisionDispatcher<P, M> + 'static>)
        -> CollisionObjectsDispatcher<P, M, T> {
        CollisionObjectsDispatcher {
            signal:           ContactSignal::new(),
            pairs:            HashMap::new(PairTWHash::new()),
            shape_dispatcher: shape_dispatcher
        }
    }

    /// Updates the contact pairs.
    pub fn update(&mut self, objects: &UidRemap<CollisionObject<P, M, T>>, timestamp: usize) {
        for e in self.pairs.elements_mut().iter_mut() {
            let co1 = &objects[e.key.first];
            let co2 = &objects[e.key.second];

            if co1.timestamp == timestamp || co2.timestamp == timestamp {
                let had_colls = e.value.num_colls() != 0;

                e.value.update(&*self.shape_dispatcher,
                               &co1.position, &**co1.shape,
                               &co2.position, &**co2.shape);

                if e.value.num_colls() == 0 {
                    if had_colls {
                        self.signal.trigger_contact_signal(&co1.data, &co2.data, false);
                    }
                }
                else {
                    if !had_colls {
                        self.signal.trigger_contact_signal(&co1.data, &co2.data, true)
                    }
                }
            }
        }
    }

    /// Iterates through all the contact pairs.
    #[inline(always)]
    pub fn contact_pairs<F>(&self,
                         objects: &UidRemap<CollisionObject<P, M, T>>,
                         mut f: F)
          where F: FnMut(&T, &T, &CollisionAlgorithm<P, M>) {
        for e in self.pairs.elements().iter() {
            let co1 = &objects[e.key.first];
            let co2 = &objects[e.key.second];

            f(&co1.data, &co2.data, &e.value)
        }
    }

    /// Calls a closures on each contact between two objects.
    #[inline(always)]
    pub fn contacts<F>(&self,
                    objects: &UidRemap<CollisionObject<P, M, T>>,
                    mut f: F)
          where F: FnMut(&T, &T, &Contact<P>) {
        // FIXME: avoid allocation.
        let mut collector = Vec::new();

        for e in self.pairs.elements().iter() {
            let co1 = &objects[e.key.first];
            let co2 = &objects[e.key.second];

            e.value.colls(&mut collector);

            for c in collector[..].iter() {
                f(&co1.data, &co2.data, c)
            }

            collector.clear();
        }
    }

    /// Registers a handler for contact start/stop events.
    pub fn register_contact_signal_handler(&mut self,
                                           name: &str,
                                           handler: Box<ContactSignalHandler<T> + 'static>) {
        self.signal.register_contact_signal_handler(name, handler)
    }

    /// Unregisters a handler for contact start/stop events.
    pub fn unregister_contact_signal_handler(&mut self, name: &str) {
        self.signal.unregister_contact_signal_handler(name)
    }

    /// Creates/removes the persistant collision detector associated to a given pair of objects.
    pub fn handle_proximity(&mut self,
                            objects: &UidRemap<CollisionObject<P, M, T>>,
                            fk1: &FastKey,
                            fk2: &FastKey,
                            started: bool) {
        let key = Pair::new(*fk1, *fk2);

        if started {
            let cd;

            {
                let co1 = &objects[*fk1];
                let co2 = &objects[*fk2];
                cd = self.shape_dispatcher.get_collision_algorithm(&co1.shape.repr(), &co2.shape.repr());
            }

            if let Some(cd) = cd {
                let _ = self.pairs.insert(key, cd);
            }
        }
        else {
            // Proximity stopped.
            match self.pairs.get_and_remove(&key) {
                Some(detector) => {
                    // Trigger the collision lost signal if there was a contact.
                    if detector.value.num_colls() != 0 {
                        let co1 = &objects[*fk1];
                        let co2 = &objects[*fk2];

                        self.signal.trigger_contact_signal(&co1.data, &co2.data, false);
                    }
                },
                None => { }
            }
        }
    }

    /// Tests if two objects can be tested for mutual collision.
    pub fn is_proximity_allowed(objects: &UidRemap<CollisionObject<P, M, T>>,
                                fk1: &FastKey,
                                fk2: &FastKey) -> bool {
        let co1 = &objects[*fk1];
        let co2 = &objects[*fk2];

        let can_move_ok = true; // XXX: ba.can_move() || bb.can_move();
        let groups_ok = co1.collision_groups.can_collide_with_groups(&co2.collision_groups);

        if *fk1 == *fk2 {
            can_move_ok && co1.collision_groups.can_collide_with_self()
        }
        else {
            can_move_ok && groups_ok
        }
    }
}
