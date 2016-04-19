# Change Log
All notable changes to `ncollide`, starting with the version 0.8.0 will be
documented here.

This project adheres to [Semantic Versioning](http://semver.org/).

## [0.9.0] - WIP
## Modified
    * `::implicit_shape_aabb(...)` becomes `::support_map_aabb(...)`
    * `CompositeShape::aabb_at(...)` now returns an AABB by-value (instead of
      by-ref).
    * `PointQuery::distance_to_point(...)` now has a `solid` flag as well.

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
