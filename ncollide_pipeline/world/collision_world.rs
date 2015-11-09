use std::ops::Mul;
use std::sync::Arc;
use std::vec::IntoIter;
use na::{Translate, Cross, Translation, Rotation};
use math::{Scalar, Point, Vect, Isometry};
use utils::data::uid_remap::{UidRemap, FastKey};
use entities::inspection::Repr;
use entities::bounding_volume::{self, AABB};
use queries::ray::{RayCast, Ray, RayIntersection};
use queries::point::PointQuery;
use narrow_phase::{BasicCollisionDispatcher, ContactSignalHandler};
use broad_phase::{BroadPhase, DBVTBroadPhase};
use world::{CollisionObjectsDispatcher, CollisionObject, CollisionGroups, ContactPairs, Contacts};

use na::{Pnt2, Pnt3, Iso2, Iso3};

// FIXME: be generic wrt the BV?
/// Type of the broad phase trait-object used by the collision world.
pub type BroadPhaseObject<P> = Box<BroadPhase<P, AABB<P>, FastKey> + 'static>;

/// A world that handles collision objects.
pub struct CollisionWorld<P, M, T> {
    objects:       UidRemap<CollisionObject<P, M, T>>,
    broad_phase:   BroadPhaseObject<P>,
    narrow_phase:  CollisionObjectsDispatcher<P, M, T>,
    pos_to_update: Vec<(FastKey, M)>,
    objects_to_remove: Vec<usize>,
    timestamp:     usize
    // FIXME: allow modification of the other properties too.
}

impl<P, M, T> CollisionWorld<P, M, T>
    where P: Point,
          P::Vect: Translate<P> + Cross,
          <P::Vect as Cross>::CrossProductType: Vect<Scalar = <P::Vect as Vect>::Scalar> +
                                                Mul<<P::Vect as Vect>::Scalar, Output = <P::Vect as Cross>::CrossProductType>, // FIXME: why do we need this?
          M:  Isometry<P, P::Vect> + Translation<P::Vect> + Rotation<<P::Vect as Cross>::CrossProductType> {
    /// Creates a new collision world.
    // FIXME: use default values for `margin` and `prediction` and allow their modification by the
    // user ?
    pub fn new(margin:     <P::Vect as Vect>::Scalar,
               prediction: <P::Vect as Vect>::Scalar,
               small_uids: bool)
               -> CollisionWorld<P, M, T> {
        let objects = UidRemap::new(small_uids);
        let sdispatcher = Box::new(BasicCollisionDispatcher::new(prediction));
        let broad_phase = Box::new(DBVTBroadPhase::<P, AABB<P>, FastKey>::new(margin, true));
        let narrow_phase = CollisionObjectsDispatcher::new(sdispatcher);

        CollisionWorld {
            objects:       objects,
            broad_phase:   broad_phase,
            narrow_phase:  narrow_phase,
            pos_to_update: Vec::new(),
            objects_to_remove: Vec::new(),
            timestamp:     0
        }
    }

    /// Adds a collision object to the world.
    pub fn add(&mut self,
               uid: usize,
               position: M,
               shape: Arc<Box<Repr<P, M>>>,
               collision_groups: CollisionGroups,
               data: T) {
        // FIXME: test that we did not add this object already ?

        let mut collision_object = CollisionObject::new(position, shape, collision_groups, data);
        collision_object.timestamp = self.timestamp;
        let aabb = bounding_volume::aabb(&**collision_object.shape, &collision_object.position);
        let fk = self.objects.insert(uid, collision_object).0;
        self.broad_phase.defered_add(fk.uid(), aabb, fk)
    }

    /// Remove a collision object from the world.
    pub fn remove(&mut self, uid: usize) {
        // mark the object to be removed
        self.objects_to_remove.push(uid);
    }

    /// Updates the collision world.
    ///
    /// This executes the whole collision detection pipeline: the broad phase first, then the
    /// narrow phase.
    pub fn update(&mut self) {
        self.perform_position_update();
        self.perform_broad_phase();
        self.perform_narrow_phase();
        
        // clean up objects that have been marked as removed
        while self.objects_to_remove.len() > 0 {
			let uid = self.objects_to_remove.pop().expect("no uid found to remove");
			// remove objects marked to as removed
			if let Some((fk, _)) = self.objects.remove(uid) {
		        self.broad_phase.defered_remove(fk.uid());
		    }
		}
    }

    /// Sets the position the collision object attached to the specified object will have during
    /// the next update.
    pub fn defered_set_position(&mut self, uid: usize, pos: M) {
        if let Some(fk) = self.objects.get_fast_key(uid) {
            self.pos_to_update.push((fk, pos))
        }
    }

    /// Registers a handler for contact start/stop events.
    pub fn register_contact_signal_handler<H>(&mut self, name: &str, handler: H)
        where H: ContactSignalHandler<T> + 'static {
        self.narrow_phase.register_contact_signal_handler(name, Box::new(handler))
    }

    /// Unregisters a handler for contact start/stop events.
    pub fn unregister_contact_signal_handler(&mut self, name: &str) {
        self.narrow_phase.unregister_contact_signal_handler(name)
    }

    /// Executes the position updates.
    pub fn perform_position_update(&mut self) {
        for &(ref fk, ref pos) in self.pos_to_update.iter() {
            if let Some(co) = self.objects.get_fast_mut(fk) {
                co.position = pos.clone();
                co.timestamp = self.timestamp;
                self.broad_phase.defered_set_bounding_volume(fk.uid(), bounding_volume::aabb(&**co.shape, pos));
            }
        }

        self.pos_to_update.clear();
    }

    /// Executes the broad phase of the collision detection pipeline.
    ///
    /// Not that this does not take in account the changes made to the collision updates after the
    /// last `.perform_position_update()` call.
    pub fn perform_broad_phase(&mut self) {
        let bf = &mut self.broad_phase;
        let nf = &mut self.narrow_phase;
        let objs = &self.objects;

        bf.update(
            &mut |b1, b2| CollisionObjectsDispatcher::is_proximity_allowed(objs, b1, b2),
            &mut |b1, b2, started| nf.handle_proximity(objs, b1, b2, started)
        );
    }

    /// Executes the narrow phase of the collision detection pipeline.
    pub fn perform_narrow_phase(&mut self) {
        self.narrow_phase.update(&self.objects, self.timestamp);
        self.timestamp = self.timestamp + 1;
    }

    /// Iterates through all the contact pairs detected since the last update.
    #[inline]
    pub fn contact_pairs(&self) -> ContactPairs<P, M, T> {
        self.narrow_phase.contact_pairs(&self.objects)
    }

    /// Iterates through every contact detected since the last update.
    #[inline]
    pub fn contacts(&self) -> Contacts<P, M, T> {
        self.narrow_phase.contacts(&self.objects)
    }

    /// Computes the interferences between every rigid bodies on this world and a ray.
    #[inline]
    pub fn interferences_with_ray<'a>(&'a self, ray: &'a Ray<P>, groups: &'a CollisionGroups)
        -> InterferencesWithRay<'a, P, M, T> {
        // FIXME: avoid allocation.
        let mut fks = Vec::new();

        self.broad_phase.interferences_with_ray(ray, &mut fks);

        InterferencesWithRay {
            ray:     ray,
            groups:  groups,
            objects: &self.objects,
            idx:     fks.into_iter()
        }
    }

    /// Computes the interferences between every rigid bodies of a given broad phase, and a point.
    #[inline]
    pub fn interferences_with_point<'a>(&'a self, point: &'a P, groups: &'a CollisionGroups)
        -> InterferencesWithPoint<'a, P, M, T> {
        // FIXME: avoid allocation.
        let mut fks = Vec::new();

        self.broad_phase.interferences_with_point(point, &mut fks);

        InterferencesWithPoint {
            point:   point,
            groups:  groups,
            objects: &self.objects,
            idx:     fks.into_iter()
        }
    }

    /// Computes the interferences between every rigid bodies of a given broad phase, and a aabb.
    #[inline]
    pub fn interferences_with_aabb<'a>(&'a self, aabb: &'a AABB<P>, groups: &'a CollisionGroups)
        -> InterferencesWithAABB<'a, P, M, T> {
        // FIXME: avoid allocation.
        let mut fks = Vec::new();

        self.broad_phase.interferences_with_bounding_volume(aabb, &mut fks);

        InterferencesWithAABB {
            groups:  groups,
            objects: &self.objects,
            idx:     fks.into_iter()
        }
    }
}

/// Iterator through all the objects on the world that intersect a specific ray.
pub struct InterferencesWithRay<'a, P: 'a + Point, M: 'a, T: 'a> {
    ray:     &'a Ray<P>,
    objects: &'a UidRemap<CollisionObject<P, M, T>>,
    groups:  &'a CollisionGroups,
    idx:     IntoIter<&'a FastKey>,
}

impl<'a, P, M, T> Iterator for InterferencesWithRay<'a, P, M, T>
    where P: Point,
          M: Isometry<P, P::Vect> + Translation<P::Vect> {
    type Item = (&'a CollisionObject<P, M, T>, RayIntersection<P::Vect>);

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        while let Some(id) = self.idx.next() {
            let co = &self.objects[*id];

            if co.collision_groups.can_collide_with_groups(self.groups) {
                let inter = co.shape.toi_and_normal_with_ray(&co.position, self.ray, true);

                if let Some(inter) = inter {
                    return Some((co, inter))
                }
            }
        }

        None
    }
}

/// Iterator through all the objects on the world that intersect a specific point.
pub struct InterferencesWithPoint<'a, P: 'a, M: 'a, T: 'a> {
    point:   &'a P,
    objects: &'a UidRemap<CollisionObject<P, M, T>>,
    groups:  &'a CollisionGroups,
    idx:     IntoIter<&'a FastKey>,
}

impl<'a, P, M, T> Iterator for InterferencesWithPoint<'a, P, M, T>
    where P: Point,
          M: Isometry<P, P::Vect> + Translation<P::Vect> {
    type Item = &'a CollisionObject<P, M, T>;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        while let Some(id) = self.idx.next() {
            let co = &self.objects[*id];

            if co.collision_groups.can_collide_with_groups(self.groups) &&
               co.shape.contains_point(&co.position, self.point) {
                return Some(co)
            }
        }

        None
    }
}

/// Iterator through all the objects on the world which bounding volume intersects a specific AABB.
pub struct InterferencesWithAABB<'a, P: 'a, M: 'a, T: 'a> {
    objects: &'a UidRemap<CollisionObject<P, M, T>>,
    groups:  &'a CollisionGroups,
    idx:     IntoIter<&'a FastKey>,
}

impl<'a, P, M, T> Iterator for InterferencesWithAABB<'a, P, M, T> {
    type Item = &'a CollisionObject<P, M, T>;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        while let Some(id) = self.idx.next() {
            let co = &self.objects[*id];

            if co.collision_groups.can_collide_with_groups(self.groups) {
                return Some(co)
            }
        }

        None
    }
}

/// 2D collision world containing objects of type `T`.
pub type CollisionWorld2<N, T> = CollisionWorld<Pnt2<N>, Iso2<N>, T>;
/// 3D collision world containing objects of type `T`.
pub type CollisionWorld3<N, T> = CollisionWorld<Pnt3<N>, Iso3<N>, T>;
