use std::rc::Rc;
use std::cell::{Ref, RefCell};
use na::{Translate, Cross, Rotation};
use bounding_volume::AABB;
use narrow_phase::{ShapeShapeDispatcher, ShapeShapeCollisionDetector};
use geometry::Contact;
use ray::{Ray, RayIntersection};
use point::PointQuery;
use math::{Scalar, Point, Vect, Isometry};
use utils::data::has_uid::HasUid;
use utils::data::has_uid_map::{HasUidMap, FastKey};
use broad_phase::{BroadPhase, ProximityFilter, DBVTBroadPhase, ProximitySignalHandler};
use narrow_phase::ContactSignalHandler;
use world::{CollisionObjectsDispatcher, CollisionObjectsProximityFilter, CollisionObject,
            AbstractCollisionDetector};

use na::{Pnt2, Pnt3, Vec2, Vec3, Iso2, Iso3};

// FIXME: be generic wrt the BV?

pub type BroadPhaseObject<P, V> = Box<BroadPhase<P, V, FastKey, AABB<P>> + 'static>;
pub type CollisionObjectRegister<N, P, V, M, O> = Rc<RefCell<HasUidMap<O, CollisionObject<N, P, V, M>>>>;
type CollisionObjectRegisterRef<'a, N, P, V, M, O> = Ref<'a, HasUidMap<O, CollisionObject<N, P, V, M>>>;

/// A world that handles collision objects.
pub struct CollisionWorld<N, P, V, M, O> {
    objects:       CollisionObjectRegister<N, P, V, M, O>,
    broad_phase:   BroadPhaseObject<P, V>,
    narrow_phase:  Rc<RefCell<CollisionObjectsDispatcher<N, P, V, M, O>>>,
    pos_to_update: Vec<(uint, FastKey, M)>,
    timestamp:     uint
    // FIXME: allow modification of the other properties too.
}

impl<N, P, V, AV, M, O> CollisionWorld<N, P, V, M, O>
    where N:  Scalar,
          P:  Point<N, V>,
          V:  Vect<N> + Translate<P> + Cross<AV>,
          AV: Vect<N>,
          M:  Isometry<N, P, V> + Rotation<AV>,
          O:  'static + HasUid {
    /// Creates a new collision world.
    // FIXME: use default values for `margin` and `prediction` and allow their modification by the
    // user ?
    pub fn new(margin: N, prediction: N) -> CollisionWorld<N, P, V, M, O> {
        let objects = Rc::new(RefCell::new(HasUidMap::new()));
        let sdispatcher = Rc::new(ShapeShapeDispatcher::new(prediction));
        let filter = box CollisionObjectsProximityFilter::new(objects.clone()) as Box<ProximityFilter<FastKey>>;
        let mut broad_phase = box DBVTBroadPhase::new(margin, Some(filter));
        let narrow_phase = Rc::new(RefCell::new(CollisionObjectsDispatcher::new(objects.clone(), sdispatcher)));

        broad_phase.register_proximity_signal_handler(
            "ncollide_default_narrow_phase_dispatch",
            box narrow_phase.clone());

        CollisionWorld {
            objects:       objects,
            broad_phase:   broad_phase as BroadPhaseObject<P, V>,
            narrow_phase:  narrow_phase,
            pos_to_update: Vec::new(),
            timestamp:     0
        }
    }

    /// Adds a collision object to the world.
    pub fn add(&mut self, object: O, collision_object: CollisionObject<N, P, V, M>) {
        // FIXME: test that we did not add this object already ?

        let mut collision_object = collision_object;
        collision_object.set_timestamp(self.timestamp);
        let aabb = collision_object.shape.aabb(&collision_object.position);
        let fk = self.objects.borrow_mut().insert(object, collision_object).val0();
        self.broad_phase.add(fk, aabb)
    }

    /// Remove a collision object from the world.
    pub fn remove(&mut self, object: &O) {
        let fast_key = self.objects.borrow().get_fast_key(object);
        if let Some(fk) = fast_key {
            self.broad_phase.remove(&fk);
            let _ = self.objects.borrow_mut().remove(object);
        }
    }

    /// Remove a collision object from the world.
    pub fn remove_with_uid(&mut self, uid: uint) {
        self.broad_phase.remove_with_uid(uid);
        let _ = self.objects.borrow_mut().remove_with_uid(uid);
    }

    /// Updates the collision world.
    ///
    /// This executes the whole collision detection pipeline: the broad phase first, then the
    /// narrow phase.
    pub fn update(&mut self) {
        self.perform_position_update();
        self.perform_broad_phase();
        self.perform_narrow_phase();
    }

    /// Sets the position the collision object attached to the specified object will have during
    /// the next update.
    pub fn set_next_position(&mut self, object: &O, pos: M) {
        self.set_next_position_with_uid(object.uid(), pos)
    }

    /// Sets the position the collision object attached to the specified object will have during
    /// the next update.
    ///
    /// Pass the unique id of the object directly.
    pub fn set_next_position_with_uid(&mut self, uid: uint, pos: M) {
        if let Some(fk) = self.objects.borrow().get_fast_key_with_uid(uid) {
            self.pos_to_update.push((uid, fk, pos))
        }
    }

    /// Registers a handler for proximity start/stop events.
    pub fn register_proximity_signal_handler<H>(&mut self, name: &str, handler: H)
        where H: ProximitySignalHandler<O> + 'static {

        let handler = OHandlerToUidHandler::new(self.objects.clone(), handler);
        self.broad_phase.register_proximity_signal_handler(name, box handler)
    }

    /// Unregisters a handler for proximity start/stop events.
    pub fn unregister_proximity_signal_handler(&mut self, name: &str) {
        self.broad_phase.unregister_proximity_signal_handler(name)
    }

    /// Registers a handler for contact start/stop events.
    pub fn register_contact_signal_handler<H>(&mut self, name: &str, handler: H)
        where H: ContactSignalHandler<O> + 'static {
        self.narrow_phase.borrow_mut().register_contact_signal_handler(name, box handler)
    }

    /// Unregisters a handler for contact start/stop events.
    pub fn unregister_contact_signal_handler(&mut self, name: &str) {
        self.narrow_phase.borrow_mut().unregister_contact_signal_handler(name)
    }

    /// Executes the position updates.
    pub fn perform_position_update(&mut self) {
        let mut bobjects = self.objects.borrow_mut();

        for &(ref uid, ref fk, ref pos) in self.pos_to_update.iter() {
            if let Some((o, co)) = bobjects.get_fast_mut(fk) {
                // Check that this is really the correct object and not one with a recycled fast
                // key.
                if o.uid() == *uid {
                    co.position = pos.clone();
                    co.set_timestamp(self.timestamp);
                    self.broad_phase.set_next_bounding_volume(fk, co.shape.aabb(pos));
                }
            }
        }

        self.pos_to_update.clear();
    }

    /// Executes the broad phase of the collision detection pipeline.
    ///
    /// Not that this does not take in account the changes made to the collision updates after the
    /// last `.perform_position_update()` call.
    pub fn perform_broad_phase(&mut self) {
        self.broad_phase.update();
    }

    /// Executes the narrow phase of the collision detection pipeline.
    pub fn perform_narrow_phase(&mut self) {
        self.narrow_phase.borrow_mut().update(self.timestamp);
        self.timestamp = self.timestamp + 1;
    }

    /// Iterats through all the contact pairs.
    #[inline(always)]
    pub fn contact_pairs(&self, f: |&O, &O, &AbstractCollisionDetector<N, P, V, M>| -> ()) {
        self.narrow_phase.borrow().contact_pairs(f)
    }

    /// Collects every contact detected since the last update.
    #[inline(always)]
    pub fn contacts(&self, f: |&O, &O, &Contact<N, P, V>| -> ()) {
        self.narrow_phase.borrow().contacts(f)
    }

    /// Computes the interferences between every rigid bodies of a given broad phase, and a ray.
    #[inline(always)]
    pub fn interferences_with_ray<'a>(&'a mut self, ray: &Ray<P, V>, f: |&O, RayIntersection<N, V>| -> ()) {
        let mut bodies = Vec::new();

        self.broad_phase.interferences_with_ray(ray, &mut bodies);

        let bobjects = self.objects.borrow();

        for b in bodies.into_iter() {
            let &(ref o, ref co) = &bobjects[*b];

            let inter = co.shape.toi_and_normal_with_transform_and_ray(&co.position, ray, true);

            if let Some(inter) = inter {
                f(o, inter)
            }
        }
    }

    /// Computes the interferences between every rigid bodies of a given broad phase, and a point.
    #[inline(always)]
    pub fn interferences_with_point(&self, point: &P, f: |&O| -> ()) {
        let mut bodies = Vec::new();

        self.broad_phase.interferences_with_point(point, &mut bodies);

        let bobjects = self.objects.borrow();

        for b in bodies.into_iter() {
            let &(ref o, ref co) = &bobjects[*b];

            if co.shape.contains_point_with_transform(&co.position, point) {
                f(o)
            }
        }
    }

    // FIXME: replace by iterators.
    /// Computes the interferences between every rigid bodies of a given broad phase, and a aabb.
    #[inline(always)]
    pub fn interferences_with_aabb(&self, aabb: &AABB<P>, f: |&O| -> ()) {
        let mut fks = Vec::new();

        self.broad_phase.interferences_with_bounding_volume(aabb, &mut fks);

        let bobjects = self.objects.borrow();

        for fk in fks.into_iter() {
            f(bobjects[*fk].ref0())
        }
    }
}

/// 2D collision world containing objects of type `O`.
pub type CollisionWorld2<N, O> = CollisionWorld<N, Pnt2<N>, Vec2<N>, Iso2<N>, O>;
/// 3D collision world containing objects of type `O`.
pub type CollisionWorld3<N, O> = CollisionWorld<N, Pnt3<N>, Vec3<N>, Iso3<N>, O>;



struct OHandlerToUidHandler<N, P, V, M, O, H> {
    objects: CollisionObjectRegister<N, P, V, M, O>,
    handler: H
}

impl<N, P, V, M, O, H> OHandlerToUidHandler<N, P, V, M, O, H> {
    fn new(objects: CollisionObjectRegister<N, P, V, M, O>, handler: H)
        -> OHandlerToUidHandler<N, P, V, M, O, H> {
        OHandlerToUidHandler {
            objects: objects,
            handler: handler
        }
    }
}

impl<N, P, V, M, O, H> ProximitySignalHandler<FastKey> for OHandlerToUidHandler<N, P, V, M, O, H>
    where O: HasUid,
          H: ProximitySignalHandler<O> {
    fn handle_proximity(&mut self, b1: &FastKey, b2: &FastKey, started: bool) {
        let bobjects = self.objects.borrow();
        let ob1 = bobjects[*b1].ref0();
        let ob2 = bobjects[*b2].ref0();

        self.handler.handle_proximity(ob1, ob2, started)
    }
}
