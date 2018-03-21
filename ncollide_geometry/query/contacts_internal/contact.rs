use std::mem;
use utils::{GenerationalId, IdAllocator};
use shape::FeatureId;
use bounding_volume::PolyhedralCone;
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
    pub fn new(world1: P, world2: P, normal: Unit<P::Vector>, depth: P::Real) -> Self {
        Contact {
            world1,
            world2,
            normal,
            depth,
        }
    }

    /// Creates a new contact, computing automatically the penetration depth.
    #[inline]
    pub fn new_wo_depth(world1: P, world2: P, normal: Unit<P::Vector>) -> Contact<P> {
        let depth = -na::dot(normal.as_ref(), &(world2 - world1));
        Self::new(world1, world2, normal, depth)
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

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
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

    /// The normal cone at the first contact point, in the local space of the first solid.
    pub normals1: PolyhedralCone<P::Vector>,

    /// The normal cone at the second contact point, in the local space of the second solid.
    pub normals2: PolyhedralCone<P::Vector>,

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
        normals1: PolyhedralCone<P::Vector>,
        normals2: PolyhedralCone<P::Vector>,
        feature1: FeatureId,
        feature2: FeatureId,
        kinematic: ContactKinematic<P::Vector>,
        id: GenerationalId,
    ) -> Self {
        TrackedContact {
            contact,
            local1,
            local2,
            normals1,
            normals2,
            feature1,
            feature2,
            kinematic,
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
            max_life: 1,                         // FIXME: don't hard-code this.
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
        local1: P,
        local2: P,
        normals1: PolyhedralCone<P::Vector>,
        normals2: PolyhedralCone<P::Vector>,
        feature1: FeatureId,
        feature2: FeatureId,
        kinematic: ContactKinematic<P::Vector>,
        gen: &mut IdAllocator,
    ) -> bool {
        // FIXME: all this is poorly designed and quite inefficient (but OK for a first
        // non-optimized implementation).
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

        let mut matched = false;
        if closest.is_invalid() {
            closest = gen.alloc();
        } else {
            if closest_is_contact {
                self.contacts[closest_i] = TrackedContact::new(
                    contact,
                    local1,
                    local2,
                    normals1,
                    normals2,
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
            if let CacheEntryStatus::Used(used_i) = self.cached_contact_used[closest_i] {
                self.contacts[used_i] = TrackedContact::new(
                    contact,
                    local1,
                    local2,
                    normals1,
                    normals2,
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
                self.cached_contact_used[closest_i] = CacheEntryStatus::Used(self.contacts.len());
                matched = true;
            }
        }

        if is_deepest {
            self.deepest = self.contacts.len();
        }
        let tracked = TrackedContact::new(
            contact,
            local1,
            local2,
            normals1,
            normals2,
            feature1,
            feature2,
            kinematic,
            closest,
        );
        self.contacts.push(tracked);
        return matched;
    }
    /*
    pub fn push(
        &mut self,
        contact: Contact<P>,
        local1: P,
        local2: P,
        normals1: PolyhedralCone<P::Vector>,
        normals2: PolyhedralCone<P::Vector>,
        feature1: FeatureId,
        feature2: FeatureId,
        kinematic: ContactKinematic<P::Vector>,
        gen: &mut IdAllocator,
    ) -> bool {
        let mut id = GenerationalId::invalid();
        let mut matched = false;

        for i in 0..self.cache.len() {
            if self.cached_contact_used[i].is_obsolete() && self.cache[i].feature1 == feature1
                && self.cache[i].feature2 == feature2
            {
                self.cached_contact_used[i] = CacheEntryStatus::Used(self.contacts.len());
                id = self.cache[i].id;
                matched = true;
            }
        }

        if id.is_invalid() {
            id = gen.alloc();
        }

        let tracked = TrackedContact::new(
            contact,
            local1,
            local2,
            normals1,
            normals2,
            feature1,
            feature2,
            kinematic,
            id,
        );
        self.contacts.push(tracked);
        matched
    }*/
}
