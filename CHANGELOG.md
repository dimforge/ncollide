# Change Log
All notable changes to `ncollide`, starting with the version 0.8.0 will be
documented here.

This project adheres to [Semantic Versioning](http://semver.org/).

## [0.12.0] - WIP
### Modified
    * The `ShapeHandle` is now a `Arc<Shape<P, M>>` instead of `Arc<Box<Shape<P, M>>>`
    (this removes one allocation indirection).
    * Update to nalgebra 0.12.

## [0.11.0]
    * Update to nalgebra 0.11.0.

## [0.10.0]
### Added
    * Re-export `Ray2`, `Ray3`, `RayIntersection2`, and `RayIntersection3` on
      the `query` module.
### Modified
    * Rename `.perform_removals_and_broad_phase()` -> `.perform_additions_removals_and_broad_phase()`.
    * Rename the collision world method `.add()` to `.deferred_add()`.
    * The collision world `.deferred_set_position()` now fails with a
      meaningful error when the user attempts to set the position of an object
      not actually added (including those that have been `.deferred_add()`-ed
      without a subsequent `.update()`.


## [0.9.0]
### Added
    * Added 2D and 3D testbeds (available on crates.io as `ncollide_testbed2d` and `ncollide_testbed3d`).
    * Added a method to the narrow phase to retrieve all the proximity pairs.
    * Added a method to the collision world to retrieve all the proximity pairs.
    * Added a method to the collision world to retrieve the collision object
    from its identifier.
### Modified
    * Merge the `ncollide_queries` and `ncollide_entities` crates into
      `ncollide_geometry`.
    * Rename the `geometry` module to `query`.
    * `::implicit_shape_aabb(...)` becomes `::support_map_aabb(...)`
    * `CompositeShape::aabb_at(...)` now returns an AABB by-value (instead of
      by-ref).
    * `PointQuery::distance_to_point(...)` now has a `solid` flag as well.
    * Point queries result now indicates if the point was inside of the object
      or not by returning a `PointProjection` structure instead of just the
      point.
    * Rename `CollisionGroups::can_collide_with` to `CollisionGroups::can_interact_with`.
    * Rename `NarrowPhase::handle_proximity` to `NarrowPhase::handle_interaction`.
    * Rename all `.*CollisionDetector` to `.*ContactGenerator`. Methods have
      been renamed as well (e.g. `.get_collision_algorithm` becomes
      `.get_contact_algorithm`, `.colls` becomes `.contacts`, etc.)
    * Rename `CollisionQueryType` to `GeometricQueryType`.
    * Moved the `ray` and `point` modules into the `query` module. Also, they
      are renamed `ray_internal` and `point_internal`.
### Removed
    * Removed the `CompositeShape::len()` method.

## [0.8.0]
### Added
    * Added proximity queries, including persistant proximity detector and
      algorithm dispatcher.
    * Added methods to set directly collision group membership/whitelist/blacklist.
### Modified
    * The last type parameter of the `BVTCostFn` trait (the user-defined data
      return by leaves) is now an associated type.
    * The shape handles `Arc<Box<Repr<P, M>>>` are now wrapped into a structure
      with a more explicit name: `ShapeHandle<P, M>`.
    * Renamed `Convex` to `ConvexHull`
    * Swapped the first two arguments of `CompositeShape::map_transformed_part_at`.
    * All fields of `Polyline` are now private. Added corresponding accessors.
