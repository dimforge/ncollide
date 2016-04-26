# Change Log
All notable changes to `ncollide`, starting with the version 0.8.0 will be
documented here.

This project adheres to [Semantic Versioning](http://semver.org/).

## [0.9.0] - WIP
### Added
    * Added a method to the narrow phase to retrieve all the proximity pairs.
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
