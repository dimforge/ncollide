use std::mem;
use utils::{GenerationalId, IdAllocator};
use shape::FeatureId;
use math::{Isometry, Point};

use na::{self, Real, Unit};

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

#[derive(Copy, Clone, Debug)]
pub enum ContactKinematic<V> {
    PlanePoint,
    PointPlane,
    PointPoint,
    PointLine(Unit<V>),
    LinePoint(Unit<V>),
    LineLine(Unit<V>, Unit<V>),
    Unknown,
}

#[derive(Clone, Debug)]
pub struct TrackedContact<P: Point> {
    pub contact: Contact<P>,

    /// The contact location in the local space of the first object.
    pub local1: P,

    /// The contact location in the local space of the second object.
    pub local2: P,

    /// The contact normal in the local space of the first object.
    pub normal1: Unit<P::Vector>,

    /// The contact normal in the local space of the second object.
    pub normal2: Unit<P::Vector>,

    pub feature1: FeatureId,
    pub feature2: FeatureId,
    pub kinematic: ContactKinematic<P::Vector>,
    pub id: GenerationalId,
}

impl<P: Point> TrackedContact<P> {
    pub fn new(
        contact: Contact<P>,
        local1: P,
        local2: P,
        normal1: Unit<P::Vector>,
        normal2: Unit<P::Vector>,
        feature1: FeatureId,
        feature2: FeatureId,
        kinematic: ContactKinematic<P::Vector>,
        id: GenerationalId,
    ) -> Self {
        TrackedContact {
            contact,
            local1,
            local2,
            normal1,
            normal2,
            feature1,
            feature2,
            kinematic,
            id,
        }
    }

    pub fn new_with_transforms<M: Isometry<P>>(
        m1: &M,
        m2: &M,
        contact: Contact<P>,
        feature1: FeatureId,
        feature2: FeatureId,
        kinematic: ContactKinematic<P::Vector>,
        id: GenerationalId,
    ) -> Self {
        let local1 = m1.inverse_transform_point(&contact.world1);
        let local2 = m2.inverse_transform_point(&contact.world2);
        let normal1 = Unit::new_unchecked(m1.inverse_transform_vector(&*contact.normal));
        let normal2 = Unit::new_unchecked(m2.inverse_transform_vector(&-*contact.normal));

        Self::new(
            contact,
            local1,
            local2,
            normal1,
            normal2,
            feature1,
            feature2,
            kinematic,
            id,
        )
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
    deepest: usize,
    contacts: Vec<TrackedContact<P>>,
    cache: Vec<TrackedContact<P>>,
    cached_contact_used: Vec<Option<usize>>,
}

impl<P: Point> ContactManifold<P> {
    pub fn new() -> Self {
        ContactManifold {
            deepest: 0,
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

    pub fn deepest_contact(&self) -> &TrackedContact<P> {
        &self.contacts[self.deepest]
    }

    pub fn save_cache_and_clear(&mut self, gen: &mut IdAllocator) {
        for (valid, c) in self.cached_contact_used.iter().zip(self.cache.iter()) {
            if !valid.is_some() {
                gen.free(c.id)
            }
        }

        mem::swap(&mut self.contacts, &mut self.cache);

        for val in &mut self.cached_contact_used {
            *val = None;
        }

        self.cached_contact_used.resize(self.cache.len(), None);
        self.contacts.clear();
        self.deepest = 0;
    }

    pub fn push<M: Isometry<P>>(
        &mut self,
        m1: &M,
        m2: &M,
        contact: Contact<P>,
        feature1: FeatureId,
        feature2: FeatureId,
        gen: &mut IdAllocator,
    ) -> bool {
        let local1 = m1.inverse_transform_point(&contact.world1);
        let mut closest = GenerationalId::invalid();
        let mut closest_i = 0;
        let mut closest_is_contact = false;
        let mut closest_dist: P::Real = na::convert(0.02 * 0.02); // FIXME: don't hard-code this.

        for i in 0..self.cache.len() {
            /*if !self.cached_contact_used[i] && self.cache[i].feature1 == feature1
                && self.cache[i].feature2 == feature2*/

            let dist = na::distance_squared(&local1, &self.cache[i].local1);
            if dist < closest_dist {
                closest_dist = dist;
                closest = self.cache[i].id;
                closest_i = i;
            }
        }

        for i in 0..self.contacts.len() {
            /*if !self.cached_contact_used[i] && self.cache[i].feature1 == feature1
                && self.cache[i].feature2 == feature2*/

            let dist = na::distance_squared(&local1, &self.contacts[i].local1);
            if dist < closest_dist {
                closest_is_contact = true;
                closest_dist = dist;
                closest = self.contacts[i].id;
                closest_i = i;
            }
        }

        let is_deepest =
            self.contacts.len() == 0 || contact.depth > self.contacts[self.deepest].contact.depth;

        let kinematic = ContactKinematic::Unknown;
        let mut matched = false;
        if closest.is_invalid() {
            closest = gen.alloc();
        } else {
            if closest_is_contact {
                self.contacts[closest_i] = TrackedContact::new_with_transforms(
                    m1,
                    m2,
                    contact,
                    feature1,
                    feature2,
                    kinematic,
                    closest,
                );
                if is_deepest {
                    self.deepest = closest_i;
                }
                return false;
            }
            if let Some(used_i) = self.cached_contact_used[closest_i] {
                self.contacts[used_i] = TrackedContact::new_with_transforms(
                    m1,
                    m2,
                    contact,
                    feature1,
                    feature2,
                    kinematic,
                    closest,
                );

                if is_deepest {
                    self.deepest = used_i;
                }
                return false;
            } else {
                self.cached_contact_used[closest_i] = Some(self.contacts.len());
                matched = true;
            }
        }

        if is_deepest {
            self.deepest = self.contacts.len();
        }
        let tracked = TrackedContact::new_with_transforms(
            m1,
            m2,
            contact,
            feature1,
            feature2,
            kinematic,
            closest,
        );
        self.contacts.push(tracked);
        return matched;
    }
    /*
    pub fn push<M: Isometry<P>>(
        &mut self,
        m1: &M,
        m2: &M,
        contact: Contact<P>,
        feature1: FeatureId,
        feature2: FeatureId,
        gen: &mut IdAllocator,
    ) {
        let mut id = GenerationalId::invalid();

        for i in 0..self.cache.len() {
            if self.cached_contact_used[i].is_none() && self.cache[i].feature1 == feature1
                && self.cache[i].feature2 == feature2
            {
                self.cached_contact_used[i] = Some(self.contacts.len());
                id = self.cache[i].id;
            }
        }

        if id.is_invalid() {
            id = gen.alloc();
        }

        let tracked = TrackedContact::new_with_transforms(m1, m2, contact, feature1, feature2, id);
        self.contacts.push(tracked)
    }*/
}
