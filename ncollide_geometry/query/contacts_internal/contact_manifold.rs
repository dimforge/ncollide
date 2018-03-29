use std::mem;
use utils::{GenerationalId, IdAllocator};
use shape::FeatureId;
use bounding_volume::PolyhedralCone;
use query::{Contact, ContactKinematic, TrackedContact};
use math::{Isometry, Point};

use na::{self, Real, Unit};

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
enum CacheEntryStatus {
    Used(usize),
    Unused(usize),
}

impl CacheEntryStatus {
    fn is_used(&self) -> bool {
        match *self {
            CacheEntryStatus::Used(..) => true,
            _ => false,
        }
    }

    fn is_obsolete(&self) -> bool {
        match *self {
            CacheEntryStatus::Unused(0) => true,
            _ => false,
        }
    }
}

#[derive(Clone, Debug)]
pub struct ContactManifold<P: Point> {
    deepest: usize,
    contacts: Vec<TrackedContact<P>>,
    // FIXME: the cache should only contain points
    // (the contact in local-space of body 1).
    cache: Vec<TrackedContact<P>>,
    cached_contact_used: Vec<CacheEntryStatus>,
    new_cached_contact_used: Vec<CacheEntryStatus>,
    max_life: usize,
}

impl<P: Point> ContactManifold<P> {
    pub fn new() -> Self {
        ContactManifold {
            deepest: 0,
            contacts: Vec::new(),
            cache: Vec::new(),
            cached_contact_used: Vec::new(),
            new_cached_contact_used: Vec::new(), // FIXME: the existence of this buffer is ugly.
            max_life: 0,                         // FIXME: don't hard-code this.
        }
    }

    pub fn len(&self) -> usize {
        self.contacts.len()
    }

    pub fn contacts(&self) -> &[TrackedContact<P>] {
        &self.contacts[..]
    }

    pub fn deepest_contact_id(&self) -> usize {
        self.deepest
    }

    pub fn deepest_contact(&self) -> Option<&TrackedContact<P>> {
        if self.len() != 0 {
            Some(&self.contacts[self.deepest])
        } else {
            None
        }
    }

    pub fn save_cache_and_clear(&mut self, gen: &mut IdAllocator) {
        for (valid, c) in self.cached_contact_used.iter_mut().zip(self.cache.iter()) {
            if valid.is_obsolete() {
                gen.free(c.id)
            }
        }

        mem::swap(&mut self.contacts, &mut self.cache);

        self.new_cached_contact_used.clear();
        self.new_cached_contact_used
            .resize(self.cache.len(), CacheEntryStatus::Unused(self.max_life));

        for (valid, c) in self.cached_contact_used
            .drain(..)
            .zip(self.contacts.drain(..))
        {
            match valid {
                CacheEntryStatus::Unused(life) if life > 0 => {
                    self.cache.push(c);
                    self.new_cached_contact_used
                        .push(CacheEntryStatus::Unused(life - 1));
                }
                _ => {}
            }
        }

        mem::swap(
            &mut self.new_cached_contact_used,
            &mut self.cached_contact_used,
        );

        self.contacts.clear();
        self.deepest = 0;
    }

    pub fn push(
        &mut self,
        contact: Contact<P>,
        kinematic: ContactKinematic<P>,
        gen: &mut IdAllocator,
    ) -> bool {
        // FIXME: all this is poorly designed and quite inefficient (but OK for a first
        // non-optimized implementation).
        let mut closest = GenerationalId::invalid();
        let mut closest_i = 0;
        let mut closest_is_contact = false;
        let mut closest_dist: P::Real = na::convert(0.02 * 0.02); // FIXME: don't hard-code this.

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
    pub fn push(
        &mut self,
        contact: Contact<P>,
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
