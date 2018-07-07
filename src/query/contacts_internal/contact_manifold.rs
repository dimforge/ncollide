use std::mem;
use na::{self, Real};
use utils::{GenerationalId, IdAllocator};
use query::{Contact, ContactKinematic, TrackedContact};


#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum CacheEntryStatus {
    Used(usize),
    Unused,
}

impl CacheEntryStatus {
    fn is_obsolete(&self) -> bool {
        match *self {
            CacheEntryStatus::Unused => true,
            _ => false,
        }
    }
}

/// A contact manifold.
///
/// A contat manifold is a set of contacts lying on the same plane.
/// The convex hull of those contacts are often interpreted as a contact
/// surface. This structure is responsible for matching new contacts with
/// old ones in order to perform an approximat tracking of the contact
/// points.
#[derive(Clone, Debug)]
pub struct ContactManifold<N: Real> {
    // FIXME: move those to the contact kinematic
    // even if all contacts share the same shubshape ?
    subshape_id1: usize,
    subshape_id2: usize,
    deepest: usize,
    contacts: Vec<TrackedContact<N>>,
    // FIXME: the cache should only contain points
    // (the contact in local-space of body 1).
    cache: Vec<TrackedContact<N>>,
    cached_contact_used: Vec<CacheEntryStatus>,
    new_cached_contact_used: Vec<CacheEntryStatus>,
}

impl<N: Real> ContactManifold<N> {
    /// Initializes a contact manifold without any contact.
    pub fn new() -> Self {
        ContactManifold {
            subshape_id1: 0,
            subshape_id2: 0,
            deepest: 0,
            contacts: Vec::new(),
            cache: Vec::new(),
            cached_contact_used: Vec::new(),
            new_cached_contact_used: Vec::new(), // FIXME: the existence of this buffer is ugly.
        }
    }

    /// The identifier of the first sub-shape the contacts of this manifold lie on.
    pub fn subshape_id1(&self) -> usize {
        self.subshape_id1
    }

    /// Sets the identifier of the first sub-shape the contacts of this manifold lie on.
    pub fn set_subshape_id1(&mut self, id: usize) {
        self.subshape_id1 = id
    }

    /// The identifier of the first sub-shape the contacts of this manifold lie on.
    pub fn subshape_id2(&self) -> usize {
        self.subshape_id2
    }

    /// Sets the identifier of the first sub-shape the contacts of this manifold lie on.
    pub fn set_subshape_id2(&mut self, id: usize) {
        self.subshape_id2 = id
    }

    /// The number of contacts contained by this manifold.
    pub fn len(&self) -> usize {
        self.contacts.len()
    }

    /// All the contact tracked by this manifold.
    pub fn contacts(&self) -> &[TrackedContact<N>] {
        &self.contacts[..]
    }

    /// The index of the contact with the greatest penetration depth.
    pub fn deepest_contact_id(&self) -> usize {
        self.deepest
    }

    /// The contact of this manifold with the deepest penetration depth.
    pub fn deepest_contact(&self) -> Option<&TrackedContact<N>> {
        if self.len() != 0 {
            Some(&self.contacts[self.deepest])
        } else {
            None
        }
    }

    /// Save the contacts to a cache and empty the manifold.
    pub fn save_cache_and_clear(&mut self, gen: &mut IdAllocator) {
        for (valid, c) in self.cached_contact_used.iter_mut().zip(self.cache.iter()) {
            if valid.is_obsolete() {
                gen.free(c.id)
            }
        }

        mem::swap(&mut self.contacts, &mut self.cache);

        self.new_cached_contact_used.clear();
        self.new_cached_contact_used
            .resize(self.cache.len(), CacheEntryStatus::Unused);

        mem::swap(
            &mut self.new_cached_contact_used,
            &mut self.cached_contact_used,
        );

        self.contacts.clear();
        self.deepest = 0;
    }

    /// Add a new contact to the manifold.
    ///
    /// The manifold will attempt to match this contact with another one
    /// previously added and added to the cache by the last call to
    /// `save_cache_and_clear`. The matching is done by spacial proximity, i.e.,
    /// two contacts that are sufficiently close will be given the same identifer.
    pub fn push(
        &mut self,
        contact: Contact<N>,
        kinematic: ContactKinematic<N>,
        gen: &mut IdAllocator,
    ) -> bool {
        // FIXME: all this is poorly designed and quite inefficient
        // (but OK for a first non-optimized implementation).
        let mut closest = GenerationalId::invalid();
        let mut closest_i = 0;
        let mut closest_is_contact = false;
        let mut closest_dist: N = na::convert(0.02 * 0.02); // FIXME: don't hard-code this.

        for i in 0..self.cache.len() {
            let dist = na::distance_squared(&kinematic.local1(), &self.cache[i].kinematic.local1());
            if dist < closest_dist {
                closest_dist = dist;
                closest = self.cache[i].id;
                closest_i = i;
            }
        }

        for i in 0..self.contacts.len() {
            let dist =
                na::distance_squared(&kinematic.local1(), &self.contacts[i].kinematic.local1());
            if dist < closest_dist {
                closest_is_contact = true;
                closest_dist = dist;
                closest = self.contacts[i].id;
                closest_i = i;
            }
        }

        let is_deepest =
            self.contacts.len() == 0 || contact.depth > self.contacts[self.deepest].contact.depth;

        let mut matched = false;
        if closest.is_invalid() {
            closest = gen.alloc();
        } else {
            if closest_is_contact {
                self.contacts[closest_i] = TrackedContact::new(contact, kinematic, closest);
                if is_deepest {
                    self.deepest = closest_i;
                }
                return false;
            }
            if let CacheEntryStatus::Used(used_i) = self.cached_contact_used[closest_i] {
                self.contacts[used_i] = TrackedContact::new(contact, kinematic, closest);

                if is_deepest {
                    self.deepest = used_i;
                }
                return false;
            } else {
                self.cached_contact_used[closest_i] = CacheEntryStatus::Used(self.contacts.len());
                matched = true;
            }
        }

        if is_deepest {
            self.deepest = self.contacts.len();
        }
        let tracked = TrackedContact::new(contact, kinematic, closest);
        self.contacts.push(tracked);
        return matched;
    }

    /*
     * Feature-based matching:
     *
    pub fn push(
        &mut self,
        contact: Contact<N>,
        kinematic: ContactKinematic<P>,
        gen: &mut IdAllocator,
    ) -> bool {
        let mut id = GenerationalId::invalid();
        let mut matched = false;

        for i in 0..self.cache.len() {
            if self.cached_contact_used[i].is_obsolete()
                && self.cache[i].kinematic.feature1() == kinematic.feature1()
                && self.cache[i].kinematic.feature2() == kinematic.feature2()
            {
                self.cached_contact_used[i] = CacheEntryStatus::Used(self.contacts.len());
                id = self.cache[i].id;
                matched = true;
            }
        }

        if id.is_invalid() {
            id = gen.alloc();
        }

        let tracked = TrackedContact::new(contact, kinematic, id);
        self.contacts.push(tracked);
        matched
    }
    */
}
