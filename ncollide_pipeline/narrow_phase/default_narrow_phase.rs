use utils::data::hash_map::HashMap;
use utils::data::pair::{Pair, PairTWHash};
use utils::data::uid_remap::{UidRemap, FastKey};
use queries::geometry::Proximity;
use narrow_phase::{CollisionDispatcher, CollisionAlgorithm, ContactSignal,   CollisionDetector,
                   ProximityDispatcher, ProximityAlgorithm, ProximitySignal, ProximityDetector,
                   NarrowPhase, ContactPairs};
use world::{CollisionObject, CollisionQueryType};
use math::Point;

// FIXME: move this to the `narrow_phase` module.
/// Collision detector dispatcher for collision objects.
pub struct DefaultNarrowPhase<P, M> {
    collision_dispatcher: Box<CollisionDispatcher<P, M> + 'static>,
    collision_detectors:  HashMap<Pair, CollisionAlgorithm<P, M>, PairTWHash>,

    proximity_dispatcher: Box<ProximityDispatcher<P, M> + 'static>,
    proximity_detectors:  HashMap<Pair, ProximityAlgorithm<P, M>, PairTWHash>,
}

impl<P: Point, M: 'static> DefaultNarrowPhase<P, M> {
    /// Creates a new `DefaultNarrowPhase`.
    pub fn new(collision_dispatcher: Box<CollisionDispatcher<P, M> + 'static>,
               proximity_dispatcher: Box<ProximityDispatcher<P, M> + 'static>)
               -> DefaultNarrowPhase<P, M> {
        DefaultNarrowPhase {
            collision_dispatcher: collision_dispatcher,
            collision_detectors:  HashMap::new(PairTWHash::new()),

            proximity_dispatcher: proximity_dispatcher,
            proximity_detectors:  HashMap::new(PairTWHash::new())
        }
    }
}

impl<P: Point, M: 'static, T> NarrowPhase<P, M, T> for DefaultNarrowPhase<P, M> {
    fn update(&mut self,
              objects:          &UidRemap<CollisionObject<P, M, T>>,
              contact_signal:   &mut ContactSignal<T>,
              proximity_signal: &mut ProximitySignal<T>,
              timestamp:        usize) {
        for e in self.collision_detectors.elements_mut().iter_mut() {
            let co1 = &objects[e.key.first];
            let co2 = &objects[e.key.second];

            if co1.timestamp == timestamp || co2.timestamp == timestamp {
                let had_colls = e.value.num_colls() != 0;

                e.value.update(&*self.collision_dispatcher,
                               &co1.position, co1.shape.as_ref(),
                               &co2.position, co2.shape.as_ref(),
                               co1.query_type.query_limit() + co2.query_type.query_limit());

                if e.value.num_colls() == 0 {
                    if had_colls {
                        contact_signal.trigger_contact_signal(&co1.data, &co2.data, false);
                    }
                }
                else {
                    if !had_colls {
                        contact_signal.trigger_contact_signal(&co1.data, &co2.data, true)
                    }
                }
            }
        }

        for e in self.proximity_detectors.elements_mut().iter_mut() {
            let co1 = &objects[e.key.first];
            let co2 = &objects[e.key.second];

            if co1.timestamp == timestamp || co2.timestamp == timestamp {
                let prev_prox = e.value.proximity();

                e.value.update(&*self.proximity_dispatcher,
                               &co1.position, co1.shape.as_ref(),
                               &co2.position, co2.shape.as_ref(),
                               co1.query_type.query_limit() + co2.query_type.query_limit());

                let new_prox = e.value.proximity();

                if new_prox != prev_prox {
                    proximity_signal.trigger_proximity_signal(&co1.data, &co2.data, prev_prox, new_prox);
                }
            }
        }
    }

    fn handle_proximity(&mut self,
                        contact_signal:   &mut ContactSignal<T>,
                        proximity_signal: &mut ProximitySignal<T>,
                        objects:          &UidRemap<CollisionObject<P, M, T>>,
                        fk1:              &FastKey,
                        fk2:              &FastKey,
                        started:          bool) {
        let key = Pair::new(*fk1, *fk2);
        let co1 = &objects[*fk1];
        let co2 = &objects[*fk2];

        match (co1.query_type, co2.query_type) {
            (CollisionQueryType::Contacts(_), CollisionQueryType::Contacts(_)) => {
                if started {
                    let cd = self.collision_dispatcher.get_collision_algorithm(&co1.shape.repr(), &co2.shape.repr());

                    if let Some(cd) = cd {
                        let _ = self.collision_detectors.insert(key, cd);
                    }
                }
                else {
                    // Proximity stopped.
                    match self.collision_detectors.get_and_remove(&key) {
                        Some(detector) => {
                            // Trigger the collision lost signal if there was a contact.
                            if detector.value.num_colls() != 0 {
                                contact_signal.trigger_contact_signal(&co1.data, &co2.data, false);
                            }
                        },
                        None => { }
                    }
                }
            },
            (_, CollisionQueryType::Proximity(_)) | (CollisionQueryType::Proximity(_), _) => {
                if started {
                    let cd = self.proximity_dispatcher.get_proximity_algorithm(&co1.shape.repr(), &co2.shape.repr());

                    if let Some(cd) = cd {
                        let _ = self.proximity_detectors.insert(key, cd);
                    }
                }
                else {
                    // Proximity stopped.
                    match self.proximity_detectors.get_and_remove(&key) {
                        Some(detector) => {
                            // Trigger the proximity lost signal if they were not disjoint.
                            let prev_prox = detector.value.proximity();
                            if prev_prox != Proximity::Disjoint {
                                proximity_signal.trigger_proximity_signal(&co1.data, &co2.data,
                                                                          prev_prox, Proximity::Disjoint);
                            }
                        },
                        None => { }
                    }
                }
            }
        }

    }

    fn contact_pairs<'a>(&'a self, objects: &'a UidRemap<CollisionObject<P, M, T>>)
                         -> ContactPairs<'a, P, M, T> {
        ContactPairs::new(objects, self.collision_detectors.elements().iter())
    }
}
