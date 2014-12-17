use std::rc::Rc;
use std::cell::RefCell;
use utils::data::hash_map::HashMap;
use utils::data::pair::{Pair, PairTWHash};
use utils::data::HasUid;
use utils::data::has_uid_map::FastKey;
use geometry::Contact;
use world::CollisionObjectRegister;
use broad_phase::{ProximityFilter, ProximitySignalHandler};
use narrow_phase::{ContactSignal, ContactSignalHandler, ShapeShapeDispatcher, ShapeShapeCollisionDetector};

/// Persistant collision detection algorithm used by the narrow phase of the collision world.
pub type AbstractCollisionDetector<N, P, V, M> = Box<ShapeShapeCollisionDetector<N, P, V, M> + Send>;

// FIXME: move this to the `narrow_phase` module.
/// Collision detector dispatcher for collision objects.
pub struct CollisionObjectsDispatcher<N, P, V, M, O> {
    signal:           ContactSignal<O>,
    objects:          CollisionObjectRegister<N, P, V, M, O>,
    shape_dispatcher: Rc<ShapeShapeDispatcher<N, P, V, M>>,
    pairs:            HashMap<Pair<FastKey>, AbstractCollisionDetector<N, P, V, M>, PairTWHash>
}

impl<N, P, V, M, O> CollisionObjectsDispatcher<N, P, V, M, O>
where O: HasUid + 'static {
    /// Creates a new `CollisionObjectsDispatcher`.
    pub fn new(objects:          CollisionObjectRegister<N, P, V, M, O>,
               shape_dispatcher: Rc<ShapeShapeDispatcher<N, P, V, M>>)
               -> CollisionObjectsDispatcher<N, P, V, M, O> {
        CollisionObjectsDispatcher {
            signal:           ContactSignal::new(),
            objects:          objects,
            pairs:            HashMap::new(PairTWHash::new()),
            shape_dispatcher: shape_dispatcher
        }
    }

    /// Updates the contact pairs.
    pub fn update(&mut self, timestamp: uint) {
        let bobjects = self.objects.borrow();

        for e in self.pairs.elements_mut().iter_mut() {
            let &(ref o1, ref co1) = &bobjects[e.key.first];
            let &(ref o2, ref co2) = &bobjects[e.key.second];

            if co1.timestamp() == timestamp || co2.timestamp() == timestamp {
                let had_colls = e.value.num_colls() != 0;

                e.value.update(&*self.shape_dispatcher,
                               &co1.position, &**co1.shape,
                               &co2.position, &**co2.shape);

                if e.value.num_colls() == 0 {
                    if had_colls {
                        self.signal.trigger_contact_signal(o1, o2, false)
                    }
                }
                else {
                    if !had_colls {
                        self.signal.trigger_contact_signal(o1, o2, true)
                    }
                }
            }
        }
    }

    /// Iterates through all the contact pairs.
    #[inline(always)]
    pub fn contact_pairs(&self, f: |&O, &O, &AbstractCollisionDetector<N, P, V, M>| -> ()) {
        let bobjects = self.objects.borrow();

        for e in self.pairs.elements().iter() {
            let co1 = &bobjects[e.key.first].0;
            let co2 = &bobjects[e.key.second].0;

            f(co1, co2, &e.value)
        }
    }

    /// Calls a closures on each contact between two objects.
    #[inline(always)]
    pub fn contacts(&self, f: |&O, &O, &Contact<N, P, V>| -> ()) {
        // FIXME: avoid allocation.
        let mut collector = Vec::new();

        let bobjects = self.objects.borrow();

        for e in self.pairs.elements().iter() {
            let o1 = &bobjects[e.key.first].0;
            let o2 = &bobjects[e.key.second].0;

            e.value.colls(&mut collector);

            for c in collector.iter() {
                f(o1, o2, c)
            }

            collector.clear();
        }
    }

    /// Registers a handler for contact start/stop events.
    pub fn register_contact_signal_handler(&mut self,
                                           name: &str,
                                           handler: Box<ContactSignalHandler<O> + 'static>) {
        self.signal.register_contact_signal_handler(name, handler)
    }

    /// Unregisters a handler for contact start/stop events.
    pub fn unregister_contact_signal_handler(&mut self, name: &str) {
        self.signal.unregister_contact_signal_handler(name)
    }
}

impl<N, P, V, M, O> ProximitySignalHandler<FastKey>
for Rc<RefCell<CollisionObjectsDispatcher<N, P, V, M, O>>>
where O: HasUid + 'static {
    fn handle_proximity(&mut self, fk1: &FastKey, fk2: &FastKey, started: bool) {
        let mut bself = self.borrow_mut();

        let key = Pair::new(*fk1, *fk2);

        if started {
            let cd;

            {
                let bobjects = bself.objects.borrow();
                let co1 = &bobjects[*fk1].1;
                let co2 = &bobjects[*fk2].1;
                cd = bself.shape_dispatcher.dispatch(&**co1.shape, &**co2.shape);
            }

            if let Some(cd) = cd {
                let _ = bself.pairs.insert(key, cd);
            }
        }
        else {
            // Proximity stopped.
            match bself.pairs.get_and_remove(&key) {
                Some(detector) => {
                    // Trigger the collision lost signal if there was a contact.
                    if detector.value.num_colls() != 0 {
                        // XXX: ugly, but seems necessary to avoid multiple borrow error (the
                        // compiler is too restrictive here).
                        let objects = bself.objects.clone();
                        let bobjects = objects.borrow();
                        let o1 = &bobjects[*fk1].0;
                        let o2 = &bobjects[*fk2].0;

                        bself.signal.trigger_contact_signal(o1, o2, false);
                    }
                },
                None => { }
            }
        }
    }
}

/// Collision filter for object havin a `CollisionObjectRef`.
pub struct CollisionObjectsProximityFilter<N, P, V, M, O> {
    objects: CollisionObjectRegister<N, P, V, M, O>
}

impl<N, P, V, M, O> CollisionObjectsProximityFilter<N, P, V, M, O> {
    /// Creates a new `CollisionObjectsProximityFilter`.
    pub fn new(objects: CollisionObjectRegister<N, P, V, M, O>)
            -> CollisionObjectsProximityFilter<N, P, V, M, O> {
        CollisionObjectsProximityFilter {
            objects: objects
        }
    }
}

impl<N, P, V, M, O> ProximityFilter<FastKey> for CollisionObjectsProximityFilter<N, P, V, M, O>
    where O: HasUid + 'static {
    fn is_proximity_allowed(&self, fk1: &FastKey, fk2: &FastKey) -> bool {
        let bobjects = self.objects.borrow();

        let co1 = &bobjects[*fk1].1;
        let co2 = &bobjects[*fk2].1;

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
