use crate::math::Point;
use crate::query::ContactPreprocessor;
use crate::query::{Contact, ContactKinematic, TrackedContact};
use crate::shape::FeatureId;
use na::{self, RealField};
use slab::Slab;
use std::collections::{hash_map::Entry, HashMap};

/// The technique used for contact tracking.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum ContactTrackingMode<N: RealField> {
    /// Contact tracking using features.
    /// Two contacts are considered the same if they are on the same features.
    FeatureBased,
    /// Contact tracking using distances.
    /// Two contacts are considered the same if they are closer than the given threshold.
    DistanceBased(N),
}

#[derive(Clone, Debug)]
enum ContactCache<N: RealField> {
    FeatureBased(HashMap<(FeatureId, FeatureId), usize>),
    DistanceBased(Vec<(Point<N>, usize)>, N),
}

/// A contact manifold.
///
/// A contact manifold is a set of contacts between two shapes.
/// If the shapes are convex, then the convex hull of those contacts are often interpreted as surface.
/// This structure is responsible for matching new contacts with old ones in order to perform an
/// approximate tracking of the contact points.
#[derive(Clone, Debug)]
pub struct ContactManifold<N: RealField> {
    ncontacts: usize,
    persistence: usize,
    deepest: usize,
    contacts: Slab<(TrackedContact<N>, usize)>,
    cache: ContactCache<N>,
    flip_new_contacts: bool,
}

impl<N: RealField> ContactManifold<N> {
    /// Initializes a contact manifold without any contact.
    ///
    /// The default contact tracking mode is set to `ContactTrackingMode::DistanceBased(0.02)`.
    pub fn new() -> Self {
        ContactManifold {
            ncontacts: 0,
            deepest: 0,
            persistence: 1,
            contacts: Slab::new(),
            cache: ContactCache::DistanceBased(Vec::new(), na::convert(0.02)),
            flip_new_contacts: false,
        }
    }

    /// The number of contacts contained by this manifold.
    pub fn len(&self) -> usize {
        self.ncontacts
    }

    /// All the contact tracked by this manifold.
    pub fn contacts(&self) -> impl Iterator<Item = &TrackedContact<N>> {
        let persistence = self.persistence;
        self.contacts
            .iter()
            .filter_map(move |(_, c)| if c.1 == persistence { Some(&c.0) } else { None })
    }

    /// Mutable reference to all the contact tracked by this manifold.
    pub fn contacts_mut(&mut self) -> impl Iterator<Item = &mut TrackedContact<N>> {
        let persistence = self.persistence;
        self.contacts.iter_mut().filter_map(move |(_, c)| {
            if c.1 == persistence {
                Some(&mut c.0)
            } else {
                None
            }
        })
    }

    /// The contact of this manifold with the deepest penetration depth.
    pub fn deepest_contact(&self) -> Option<&TrackedContact<N>> {
        if self.len() != 0 {
            Some(&self.contacts[self.deepest].0)
        } else {
            None
        }
    }

    /// Empty the manifold as well as its cache.
    pub fn clear(&mut self) {
        match &mut self.cache {
            ContactCache::FeatureBased(h) => h.clear(),
            ContactCache::DistanceBased(v, _) => v.clear(),
        }
        self.contacts.clear();
        self.ncontacts = 0;
    }

    /// Gets the technique currently used for tracking contacts.
    pub fn tracking_mode(&self) -> ContactTrackingMode<N> {
        match self.cache {
            ContactCache::FeatureBased(_) => ContactTrackingMode::FeatureBased,
            ContactCache::DistanceBased(_, threshold) => {
                ContactTrackingMode::DistanceBased(threshold)
            }
        }
    }

    /// Sets the technique used for tracking contacts.
    ///
    /// If the selected method is different from the current one,
    /// the current contact cache is cleared.
    pub fn set_tracking_mode(&mut self, mode: ContactTrackingMode<N>) {
        match mode {
            ContactTrackingMode::FeatureBased => {
                if let ContactCache::FeatureBased(_) = self.cache {
                    // Nothing to do.
                } else {
                    self.cache = ContactCache::FeatureBased(HashMap::new())
                }
            }
            ContactTrackingMode::DistanceBased(new_threshold) => {
                if let ContactCache::DistanceBased(_, threshold) = &mut self.cache {
                    *threshold = new_threshold;
                    return;
                }

                self.cache = ContactCache::DistanceBased(Vec::new(), new_threshold)
            }
        }
    }

    /// Save the contacts to a cache and empty the manifold.
    pub fn save_cache_and_clear(&mut self) {
        match &mut self.cache {
            ContactCache::DistanceBased(cache, _) => {
                let ctcts = &self.contacts;
                cache.retain(|c| ctcts[c.1].1 != 0);
            }
            ContactCache::FeatureBased(cache) => {
                let ctcts = &self.contacts;
                cache.retain(|_k, v| ctcts[*v].1 != 0);
            }
        }

        self.deepest = 0;
        self.ncontacts = 0;
        self.contacts.retain(|_i, c| {
            if c.1 == 0 {
                false
            } else {
                c.1 -= 1;
                true
            }
        });
    }

    // FIXME: the method taking a preprocessor should be different?
    /// Add a new contact to the manifold.
    ///
    /// The manifold will attempt to match this contact with another one
    /// previously added and added to the cache by the last call to
    /// `save_cache_and_clear`. The matching is done by spacial proximity, i.e.,
    /// two contacts that are sufficiently close will be given the same identifier.
    pub fn push(
        &mut self,
        mut contact: Contact<N>,
        mut kinematic: ContactKinematic<N>,
        tracking_pt: Point<N>,
        preprocessor1: Option<&dyn ContactPreprocessor<N>>,
        preprocessor2: Option<&dyn ContactPreprocessor<N>>,
    ) -> bool {
        if self.flip_new_contacts {
            contact.flip();
            kinematic.flip();
        }

        if let Some(pp) = preprocessor1 {
            if !pp.process_contact(&mut contact, &mut kinematic, !self.flip_new_contacts) {
                return false;
            }
        }

        if let Some(pp) = preprocessor2 {
            if !pp.process_contact(&mut contact, &mut kinematic, self.flip_new_contacts) {
                return false;
            }
        }

        let is_deepest =
            self.ncontacts == 0 || contact.depth > self.contacts[self.deepest].0.contact.depth;

        match &mut self.cache {
            ContactCache::DistanceBased(cache, threshold) => {
                let mut closest = cache.len();
                let mut closest_dist: N = *threshold * *threshold;

                for (i, cached) in cache.iter().enumerate() {
                    let dist = na::distance_squared(&tracking_pt, &cached.0);
                    if dist < closest_dist {
                        closest_dist = dist;
                        closest = i;
                    }
                }

                if closest == cache.len() {
                    let tracked = TrackedContact::new(contact, kinematic);
                    let i = self.contacts.insert((tracked, self.persistence));
                    cache.push((tracking_pt, i));
                    self.ncontacts += 1;

                    if is_deepest {
                        self.deepest = i;
                    }

                    false
                } else {
                    let contact_i = cache[closest].1;
                    if is_deepest {
                        self.deepest = contact_i;
                    }

                    let c = &mut self.contacts[contact_i];

                    if c.1 == self.persistence {
                        if contact.depth <= c.0.contact.depth {
                            // Keep the contact already in cache because it is deeper.
                            return true;
                        }
                    } else {
                        self.ncontacts += 1;
                        c.1 = self.persistence;
                    }

                    c.0.contact = contact;
                    c.0.kinematic = kinematic;
                    cache[closest].0 = tracking_pt;

                    true
                }
            }
            ContactCache::FeatureBased(cache) => {
                match cache.entry((kinematic.feature1(), kinematic.feature2())) {
                    Entry::Vacant(e) => {
                        let tracked = TrackedContact::new(contact, kinematic);
                        let i = self.contacts.insert((tracked, self.persistence));
                        let _ = e.insert(i);
                        self.ncontacts += 1;

                        if is_deepest {
                            self.deepest = i;
                        }

                        false
                    }
                    Entry::Occupied(e) => {
                        if is_deepest {
                            self.deepest = *e.get();
                        }

                        let c = &mut self.contacts[*e.get()];

                        if c.1 == self.persistence {
                            if contact.depth <= c.0.contact.depth {
                                // Keep the contact already in cache because it is deeper.
                                return true;
                            }
                        } else {
                            self.ncontacts += 1;
                            c.1 = self.persistence;
                        }

                        c.0.contact = contact;
                        c.0.kinematic = kinematic;

                        true
                    }
                }
            }
        }
    }

    /// Toggle whether `push` flips contacts
    pub(crate) fn flip_new_contacts(&mut self) {
        self.flip_new_contacts ^= true;
    }
}
