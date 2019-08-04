use na::RealField;
use std::vec::IntoIter;

use crate::math::Point;
use crate::query::{Ray, RayIntersection, RayCast, PointQuery};
use crate::bounding_volume::AABB;
use crate::pipeline::object::{CollisionObjectRef, CollisionObjectSet, CollisionGroups};
use crate::pipeline::broad_phase::BroadPhase;

/// Returns an iterator yielding all the collision objects intersecting with the given ray.
///
/// The result will only include collision objects in a group that can interact with the given `groups`.
pub fn interferences_with_ray<'a, 'b, N, Objects>(objects: &'a Objects,
                                                  broad_phase: &'a (impl BroadPhase<N, AABB<N>, Objects::CollisionObjectHandle> + ?Sized),
                                                  ray: &'b Ray<N>,
                                                  groups: &'b CollisionGroups)
                                                  -> InterferencesWithRay<'a, 'b, N, Objects>
    where N: RealField,
          Objects: CollisionObjectSet<N> {
    let mut handles = Vec::new();
    broad_phase.interferences_with_ray(ray, &mut handles);

    InterferencesWithRay {
        ray,
        groups,
        objects,
        handles: handles.into_iter(),
    }
}

/// Iterator through all the objects on the world that intersect a specific ray.
pub struct InterferencesWithRay<'a, 'b, N: RealField, Objects: CollisionObjectSet<N>> {
    ray: &'b Ray<N>,
    objects: &'a Objects,
    groups: &'b CollisionGroups,
    handles: IntoIter<&'a Objects::CollisionObjectHandle>,
}

impl<'a, 'b, N: RealField, Objects> Iterator for InterferencesWithRay<'a, 'b, N, Objects>
    where N: RealField,
          Objects: CollisionObjectSet<N> {
    type Item = (&'a Objects::CollisionObject, RayIntersection<N>);

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        while let Some(handle) = self.handles.next() {
            if let Some(co) = self.objects.collision_object(*handle) {
                if co.collision_groups().can_interact_with_groups(self.groups) {
                    let inter = co
                        .shape()
                        .toi_and_normal_with_ray(&co.position(), self.ray, true);

                    if let Some(inter) = inter {
                        return Some((co, inter));
                    }
                }
            }
        }

        None
    }
}



/// Returns an iterator yielding all the collision objects containing the given point.
///
/// The result will only include collision objects in a group that can interact with the given `groups`.
pub fn interferences_with_point<'a, 'b, N, Objects>(objects: &'a Objects,
                                                    broad_phase: &'a (impl BroadPhase<N, AABB<N>, Objects::CollisionObjectHandle> + ?Sized),
                                                    point: &'b Point<N>,
                                                    groups: &'b CollisionGroups)
                                                    -> InterferencesWithPoint<'a, 'b, N, Objects>
    where N: RealField,
          Objects: CollisionObjectSet<N> {
    let mut handles = Vec::new();
    broad_phase.interferences_with_point(point, &mut handles);

    InterferencesWithPoint {
        point,
        groups,
        objects,
        handles: handles.into_iter(),
    }
}

/// Iterator through all the objects on the world that intersect a specific point.
pub struct InterferencesWithPoint<'a, 'b, N: RealField, Objects: CollisionObjectSet<N>> {
    point: &'b Point<N>,
    objects: &'a Objects,
    groups: &'b CollisionGroups,
    handles: IntoIter<&'a Objects::CollisionObjectHandle>,
}

impl<'a, 'b, N: RealField, Objects> Iterator for InterferencesWithPoint<'a, 'b, N, Objects>
    where N: RealField,
          Objects: CollisionObjectSet<N> {
    type Item = &'a Objects::CollisionObject;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        while let Some(handle) = self.handles.next() {
            if let Some(co) = self.objects.collision_object(*handle) {
                if co.collision_groups().can_interact_with_groups(self.groups)
                    && co.shape().contains_point(&co.position(), self.point)
                {
                    return Some(co);
                }
            }
        }

        None
    }
}


/// Returns an iterator yielding all the collision objects with an AABB intersecting with the given AABB.
///
/// The result will only include collision objects in a group that can interact with the given `groups`.
pub fn interferences_with_aabb<'a, 'b, N, Objects>(objects: &'a Objects,
                                                   broad_phase: &'a (impl BroadPhase<N, AABB<N>, Objects::CollisionObjectHandle> + ?Sized),
                                                   aabb: &AABB<N>,
                                                   groups: &'b CollisionGroups)
                                                   -> InterferencesWithAABB<'a, 'b, N, Objects>
    where N: RealField,
          Objects: CollisionObjectSet<N> {
    let mut handles = Vec::new();
    broad_phase.interferences_with_bounding_volume(aabb, &mut handles);

    InterferencesWithAABB {
        groups,
        objects,
        handles: handles.into_iter(),
    }
}

/// Iterator through all the objects on the world which bounding volume intersects a specific AABB.
pub struct InterferencesWithAABB<'a, 'b, N: RealField, Objects: CollisionObjectSet<N>> {
    objects: &'a Objects,
    groups: &'b CollisionGroups,
    handles: IntoIter<&'a Objects::CollisionObjectHandle>,
}

impl<'a, 'b, N: RealField, Objects: CollisionObjectSet<N>> Iterator for InterferencesWithAABB<'a, 'b, N, Objects> {
    type Item = &'a Objects::CollisionObject;

    #[inline]
    fn next(&mut self) -> Option<Self::Item> {
        while let Some(handle) = self.handles.next() {
            if let Some(co) = self.objects.collision_object(*handle) {
                if co.collision_groups().can_interact_with_groups(self.groups) {
                    return Some(co);
                }
            }
        }

        None
    }
}
