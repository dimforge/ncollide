use std::mem;
use math::Point;
use utils::{GenerationalId, IdAllocator};
use shape::FeatureId;

use na::{Real, Unit};

/// Geometric description of a contact.
#[derive(Debug, PartialEq, Clone)]
pub struct Contact<P: Point> {
    /// Position of the contact on the first object. The position is expressed in world space.
    pub world1: P,

    /// Position of the contact on the second object. The position is expressed in world space.
    pub world2: P,

    /// Contact normal
    pub normal: Unit<P::Vector>,

    /// Penetration depth
    pub depth: P::Real,
}

impl<P: Point> Contact<P> {
    /// Creates a new contact.
    #[inline]
    pub fn new(world1: P, world2: P, normal: Unit<P::Vector>, depth: P::Real) -> Contact<P> {
        Contact {
            world1: world1,
            world2: world2,
            normal: normal,
            depth: depth,
        }
    }
}

impl<P: Point> Contact<P> {
    /// Reverts the contact normal and swaps `world1` and `world2`.
    #[inline]
    pub fn flip(&mut self) {
        mem::swap(&mut self.world1, &mut self.world2);
        self.normal = -self.normal;
    }
}

#[derive(Clone, Debug)]
pub struct TrackedContact<P: Point> {
    pub contact: Contact<P>,
    pub feature1: FeatureId,
    pub feature2: FeatureId,
    pub id: GenerationalId,
}

impl<P: Point> TrackedContact<P> {
    pub fn new(
        contact: Contact<P>,
        feature1: FeatureId,
        feature2: FeatureId,
        id: GenerationalId,
    ) -> Self {
        TrackedContact {
            contact,
            feature1,
            feature2,
            id,
        }
    }
}

/// The prediction parameters for contact determination.n
pub struct ContactPrediction<N: Real> {
    /// The linear prediction.
    pub linear: N,
    /// The angular regularization for the first solid.
    pub angular1: N,
    /// The angular regularization for the second solid.
    pub angular2: N,
}

impl<N: Real> ContactPrediction<N> {
    /// Initialize prediction parameters.
    pub fn new(linear: N, angular1: N, angular2: N) -> Self {
        ContactPrediction {
            linear,
            angular1,
            angular2,
        }
    }
}

#[derive(Clone, Debug)]
pub struct ContactManifold<P: Point> {
    contacts: Vec<TrackedContact<P>>,
    cache: Vec<TrackedContact<P>>,
    cached_contact_used: Vec<bool>,
}

impl<P: Point> ContactManifold<P> {
    pub fn new() -> Self {
        ContactManifold {
            contacts: Vec::new(),
            cache: Vec::new(),
            cached_contact_used: Vec::new(),
        }
    }

    pub fn len(&self) -> usize {
        self.contacts.len()
    }

    pub fn contacts(&self) -> &[TrackedContact<P>] {
        &self.contacts[..]
    }

    pub fn save_cache_and_clear(&mut self, gen: &mut IdAllocator) {
        for (valid, c) in self.cached_contact_used.iter().zip(self.cache.iter()) {
            if !*valid {
                gen.free(c.id)
            }
        }

        mem::swap(&mut self.contacts, &mut self.cache);

        for val in &mut self.cached_contact_used {
            *val = false;
        }

        self.cached_contact_used.resize(self.cache.len(), false);
        self.contacts.clear();
    }

    pub fn push(
        &mut self,
        contact: Contact<P>,
        feature1: FeatureId,
        feature2: FeatureId,
        gen: &mut IdAllocator,
    ) {
        let mut id = GenerationalId::invalid();

        for i in 0..self.cache.len() {
            if !self.cached_contact_used[i] && self.cache[i].feature1 == feature1
                && self.cache[i].feature2 == feature2
            {
                self.cached_contact_used[i] = true;
                id = self.cache[i].id;
            }
        }

        if id.is_invalid() {
            id = gen.alloc();
        }

        let tracked = TrackedContact::new(contact, feature1, feature2, id);
        self.contacts.push(tracked)
    }
}
