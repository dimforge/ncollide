use utils::data::hash_map::HashMap;
use utils::data::pair::{Pair, PairTWHash};
use utils::data::uid_remap::{UidRemap, FastKey};
use geometry::query::Proximity;
use narrow_phase::{ContactDispatcher, ContactAlgorithm, ContactSignal,   ContactGenerator,
                   ProximityDispatcher, ProximityAlgorithm, ProximitySignal, ProximityDetector,
                   NarrowPhase, ContactPairs, ProximityPairs};
use world::{CollisionObject, GeometricQueryType};
use math::Point;

// FIXME: move this to the `narrow_phase` module.
/// Collision detector dispatcher for collision objects.
pub struct DefaultNarrowPhase<P, M> {
    contact_dispatcher: Box<ContactDispatcher<P, M> + 'static>,
    contact_generators:  HashMap<Pair, ContactAlgorithm<P, M>, PairTWHash>,

    proximity_dispatcher: Box<ProximityDispatcher<P, M> + 'static>,
    proximity_detectors:  HashMap<Pair, ProximityAlgorithm<P, M>, PairTWHash>,
}

impl<P: Point, M: 'static> DefaultNarrowPhase<P, M> {
    /// Creates a new `DefaultNarrowPhase`.
    pub fn new(contact_dispatcher: Box<ContactDispatcher<P, M> + 'static>,
               proximity_dispatcher: Box<ProximityDispatcher<P, M> + 'static>)
               -> DefaultNarrowPhase<P, M> {
        DefaultNarrowPhase {
            contact_dispatcher: contact_dispatcher,
            contact_generators:  HashMap::new(PairTWHash::new()),

            proximity_dispatcher: proximity_dispatcher,
            proximity_detectors:  HashMap::new(PairTWHash::new())
        }
    }
}

impl<P: Point, M: 'static, T> NarrowPhase<P, M, T> for DefaultNarrowPhase<P, M> {
    fn update(&mut self,
              objects:          &UidRemap<CollisionObject<P, M, T>>,
              contact_signal:   &mut ContactSignal<P, M, T>,
              proximity_signal: &mut ProximitySignal<P, M, T>,
              timestamp:        usize) {
        for e in self.contact_generators.elements_mut().iter_mut() {
            let co1 = &objects[e.key.first];
            let co2 = &objects[e.key.second];

            if co1.timestamp == timestamp || co2.timestamp == timestamp {
                let had_contacts = e.value.num_contacts() != 0;

                e.value.update(&*self.contact_dispatcher,
                               &co1.position, co1.shape.as_ref(),
                               &co2.position, co2.shape.as_ref(),
                               co1.query_type.query_limit() + co2.query_type.query_limit());

                if e.value.num_contacts() == 0 {
                    if had_contacts {
                        contact_signal.trigger_contact_stopped_signal(&co1, &co2);
                    }
                }
                else {
                    if !had_contacts {
                        contact_signal.trigger_contact_started_signal(&co1, &co2, &e.value)
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
                    proximity_signal.trigger_proximity_signal(&co1, &co2, prev_prox, new_prox);
                }
            }
        }
    }

    fn handle_interaction(&mut self,
                          contact_signal:   &mut ContactSignal<P, M, T>,
                          proximity_signal: &mut ProximitySignal<P, M, T>,
                          objects:          &UidRemap<CollisionObject<P, M, T>>,
                          fk1:              &FastKey,
                          fk2:              &FastKey,
                          started:          bool) {
        let key = Pair::new(*fk1, *fk2);
        let co1 = &objects[*fk1];
        let co2 = &objects[*fk2];

        match (co1.query_type, co2.query_type) {
            (GeometricQueryType::Contacts(_), GeometricQueryType::Contacts(_)) => {
                if started {
                    let cd = self.contact_dispatcher.get_contact_algorithm(co1.shape.as_ref(), co2.shape.as_ref());

                    if let Some(cd) = cd {
                        let _ = self.contact_generators.insert(key, cd);
                    }
                }
                else {
                    // Proximity stopped.
                    match self.contact_generators.get_and_remove(&key) {
                        Some(detector) => {
                            // Trigger the collision lost signal if there was a contact.
                            if detector.value.num_contacts() != 0 {
                                contact_signal.trigger_contact_stopped_signal(&co1, &co2);
                            }
                        },
                        None => { }
                    }
                }
            },
            (_, GeometricQueryType::Proximity(_)) | (GeometricQueryType::Proximity(_), _) => {
                if started {
                    let cd = self.proximity_dispatcher.get_proximity_algorithm(co1.shape.as_ref(), co2.shape.as_ref());

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
                                proximity_signal.trigger_proximity_signal(&co1, &co2, prev_prox, Proximity::Disjoint);
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
        ContactPairs::new(objects, self.contact_generators.elements().iter())
    }

    fn proximity_pairs<'a>(&'a self, objects: &'a UidRemap<CollisionObject<P, M, T>>)
                           -> ProximityPairs<'a, P, M, T> {
        ProximityPairs::new(objects, self.proximity_detectors.elements().iter())
    }
}
