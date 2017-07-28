use std::vec::IntoIter;
use std::slice::Iter;
use utils::data::hash_map::Entry;
use utils::data::pair::Pair;
use utils::data::uid_remap::{UidRemap, FastKey};
use geometry::query::Contact;
use narrow_phase::{ContactAlgorithm, ContactSignal,
                   ProximityAlgorithm, ProximitySignal};
use world::CollisionObject;
use math::Point;

/// Trait implemented by the narrow phase manager.
///
/// The narrow phase manager is responsible for creating, updating and generating contact pairs
/// between objects identified by the broad phase.
pub trait NarrowPhase<P: Point, M, T> {
    /// Updates this narrow phase.
    fn update(&mut self,
              objects:          &UidRemap<CollisionObject<P, M, T>>,
              contact_signal:   &mut ContactSignal<P, M, T>,
              proximity_signal: &mut ProximitySignal<P, M, T>,
              timestamp:        usize);

    /// Called when the broad phase detects that two objects are, or stop to be, in close proximity.
    fn handle_interaction(&mut self,
                          contact_signal:   &mut ContactSignal<P, M, T>,
                          proximity_signal: &mut ProximitySignal<P, M, T>,
                          objects:          &UidRemap<CollisionObject<P, M, T>>,
                          fk1:              &FastKey,
                          fk2:              &FastKey,
                          started:          bool);

    // FIXME: the fact that the return type is imposed is not as generic as it could be.
    /// Returns all the potential contact pairs found during the broad phase, and validated by the
    /// narrow phase.
    fn contact_pairs<'a>(&'a self, objects: &'a UidRemap<CollisionObject<P, M, T>>)
                         -> ContactPairs<'a, P, M, T>;

    /// Returns all the potential proximity pairs found during the broad phase, and validated by
    /// the narrow phase.
    fn proximity_pairs<'a>(&'a self, objects: &'a UidRemap<CollisionObject<P, M, T>>)
                         -> ProximityPairs<'a, P, M, T>;
}

/// Iterator through contact pairs.
pub struct ContactPairs<'a, P: Point + 'a, M: 'a, T: 'a> {
    objects: &'a UidRemap<CollisionObject<P, M, T>>,
    pairs:   Iter<'a, Entry<Pair, ContactAlgorithm<P, M>>>
}

impl<'a, P: 'a + Point, M: 'a, T: 'a> ContactPairs<'a, P, M, T> {
    #[doc(hidden)]
    #[inline]
    pub fn new(objects: &'a UidRemap<CollisionObject<P, M, T>>,
               pairs:   Iter<'a, Entry<Pair, ContactAlgorithm<P, M>>>)
               -> ContactPairs<'a, P, M, T> {
        ContactPairs {
            objects: objects,
            pairs:   pairs
        }
    }

    /// Transforms contact-pairs iterator to an iterator through each individual contact.
    #[inline]
    pub fn contacts(self) -> Contacts<'a, P, M, T> {
        Contacts {
            objects:      self.objects,
            curr:         None,
            pairs:        self.pairs,
        }
    }
}

impl<'a, P: Point, M, T> Iterator for ContactPairs<'a, P, M, T> {
    type Item = (&'a CollisionObject<P, M, T>,
                 &'a CollisionObject<P, M, T>,
                 &'a ContactAlgorithm<P, M>);

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        match self.pairs.next() {
            Some(p) => {
                let co1 = &self.objects[p.key.first];
                let co2 = &self.objects[p.key.second];

                Some((&co1, &co2, &p.value))
            }
            None => None
        }
    }
}

/// An iterator through contacts.
pub struct Contacts<'a, P: 'a + Point, M: 'a, T: 'a> {
    objects:      &'a UidRemap<CollisionObject<P, M, T>>,
    curr:         Option<(&'a CollisionObject<P, M, T>, &'a CollisionObject<P, M, T>, IntoIter<Contact<P>>)>,
    pairs:        Iter<'a, Entry<Pair, ContactAlgorithm<P, M>>>,
}

impl<'a, P: Point, M, T> Iterator for Contacts<'a, P, M, T> {
    type Item = (&'a CollisionObject<P, M, T>, &'a CollisionObject<P, M, T>, Contact<P>);

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {

        if let Some((co1, co2, ref mut iter)) = self.curr {
            if let Some(contact) = iter.next() {
                return Some((co1, co2, contact));
            }
        }

        while let Some(p) = self.pairs.next() {
            if p.value.num_contacts() > 0 {

                let mut collector = Vec::new(); // FIXME: Avoid allocations
                p.value.contacts(&mut collector);
                let mut iter = collector.into_iter();

                let co1 = &self.objects[p.key.first];
                let co2 = &self.objects[p.key.second];
                let first = iter.next().unwrap();

                self.curr = Some((co1, co2, iter));

                return Some((co1, co2, first));
            }

        }

        None

    }
}


/// Iterator through proximity pairs.
pub struct ProximityPairs<'a, P: Point + 'a, M: 'a, T: 'a> {
    objects: &'a UidRemap<CollisionObject<P, M, T>>,
    pairs:   Iter<'a, Entry<Pair, ProximityAlgorithm<P, M>>>
}

impl<'a, P: 'a + Point, M: 'a, T: 'a> ProximityPairs<'a, P, M, T> {
    #[doc(hidden)]
    #[inline]
    pub fn new(objects: &'a UidRemap<CollisionObject<P, M, T>>,
               pairs:   Iter<'a, Entry<Pair, ProximityAlgorithm<P, M>>>)
               -> ProximityPairs<'a, P, M, T> {
        ProximityPairs {
            objects: objects,
            pairs:   pairs
        }
    }
}

impl<'a, P: Point, M, T> Iterator for ProximityPairs<'a, P, M, T> {
    type Item = (&'a CollisionObject<P, M, T>,
                 &'a CollisionObject<P, M, T>,
                 &'a ProximityAlgorithm<P, M>);

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        match self.pairs.next() {
            Some(p) => {
                let co1 = &self.objects[p.key.first];
                let co2 = &self.objects[p.key.second];

                Some((&co1, &co2, &p.value))
            }
            None => None
        }
    }
}
