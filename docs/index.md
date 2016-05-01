<center>
![ncollide](./img/logo_ncollide.svg)
</center>
<br/>
<center>
[![Travis.ci Status](https://travis-ci.org/sebcrozet/ncollide.svg?branch=master)](https://travis-ci.org/sebcrozet/ncollide)
[![Crates.io Status](http://meritbadge.herokuapp.com/ncollide)](https://crates.io/crates/ncollide)
[![License (3-Clause BSD)](https://img.shields.io/badge/license-BSD%203--Clause-blue.svg?style=flat)](http://opensource.org/licenses/BSD-3-Clause)

-----

<span class="h1 headline">2D and 3D collision detection library</span>
<div></div>
<span class="subheadline">… for the [Rust](https://www.rust-lang.org) programming language.</span>
</center>

<br>

<table>
<tr>
    <td><a href="../geometric_representations"><img src="../img/feature_complex_shapes.svg"></img></a></td>
    <td style="vertical-align:middle">
    <b><big>Complex shapes</big></b><br>
    Complex geometric queries for collision detection are available for 2D and
    3D shapes with various levels of complexity: from simple spheres to
    arbitrary triangle meshes and unions of convex shapes.
    </td>
</tr>

<tr>
    <td><a href="../bounding_volumes"><img src="../img/feature_bounding_volumes.svg"></img></a></td>
    <td style="vertical-align:middle">
    <b><big>Bounding volumes</big></b><br>
    Complex shapes may be approximated by simpler ones like AABB and bounding
    spheres. Those can be used for approximating some geometric queries
    efficiently and avoid useless exact computations.
    </td>
</tr>

<tr>
    <td><a href="../geometric_queries/#ray-casting"><img src="../img/feature_ray_casting.svg"></img></a></td>
    <td style="vertical-align:middle">
    <b><big>Ray casting</big></b><br>
    Computation of intersection between a ray and any shape exposed by
    **ncollide**.
    </td>
</tr>

<tr>
    <td><a href="../geometric_queries/#point-projection"><img src="../img/feature_point_projection.svg"></img></a></td>
    <td style="vertical-align:middle">
    <b><big>Point projection</big></b><br>
    Point containment test, distance to point, and point projection on any
    shape.
    </td>
</tr>

<tr>
    <td><a href="../geometric_queries/#contact"><img src="../img/feature_contact_points.svg"></img></a></td>
    <td style="vertical-align:middle">
    <b><big>Contact points</big></b><br>
    Find the closest points between objects in close proximity.  If they are
    penetrating, the minimal translational distance can be obtained as well!
    </td>
</tr>

<tr>
    <td><a href="../geometric_queries/#time-of-impact"><img src="../img/feature_time_of_impact.svg"></img></a></td>
    <td style="vertical-align:middle">
    <b><big>Time-of-impact</big></b><br>
    Compute the time it would take for two moving shapes to start being in
    contact.
    </td>
</tr>

<tr>
    <td><a href="../geometric_queries/#distance"><img src="../img/feature_smallest_distance.svg"></img></a></td>
    <td style="vertical-align:middle">
    <b><big>Smallest distance</big></b><br>
    Compute the global minimal distance between two shapes.
    </td>
</tr>

<tr>
    <td><a href="../bounding_volumes/#spacial-partitioning"><img src="../img/feature_spacial_partitioning.svg"></img></a></td>
    <td style="vertical-align:middle">
    <b><big>Spacial partitioning</big></b><br>
    Efficient data structures to perform geometric queries efficiently on
    thousands of objects.
    </td>
</tr>

<tr>
    <td><a href="../collision_detection_pipeline"><img src="../img/feature_collision_detection_pipeline.gif"></img></a></td>
    <td style="vertical-align:middle">
    <b><big>Collision detection pipeline</big></b><br>
    A complete and efficient collision detection pipeline including a broad
    phase and a narrow phase. Enables real-time contacts generation, proximity events,
    ray-casting and more, for scenes with hundreds of objects.
    </td>
</tr>

<tr>
    <td><a href="../mesh_generation"><img src="../img/feature_mesh_generation.png"></img></a></td>
    <td style="vertical-align:middle">
    <b><big>Mesh generation</big></b><br>
    Generate triangle meshes from smooth objects, compute their convex hull, or
    decompose them approximately into their convex components.
    </td>
</tr>


</table>

----

## Demonstrations

<big>[**nphysics**](http://nphysics.org)</big> − a 2D and 3D physics engine
available on [crates.io](http://crates.io) as the `nphysics2d` and `nphysics3d`
crates.  It completely relies on **ncollide** for contact points computation
and proximity detection. It has at least one example per collision detection
algorithm provided by **ncollide**.
<p>
<center>
<iframe width="560" height="315" src="http://www.youtube.com/embed/CANjXZ5rocI" frameborder="0" allowfullscreen></iframe>
</center>
</p>
<big>[**kiss3d**](http://kiss3d.org)</big> − a simplistic 3d graphics engine
available on [crates.io](http://crates.io). It relies on mesh generation to
render everything that is not triangular (sphere, cone, Bézier surfaces, etc.)
In particular it has a
[demo](https://github.com/sebcrozet/kiss3d/blob/master/examples/procedural.rs)
that uses most mesh generation algorithms of **ncollide**. In fact, every
single 3d illustration of the [Mesh generation](mesh_generation/index.html)
section has been rendered by **kiss3d**.
<p>
<center>
![Mesh generation rendered by kiss3d](img/kiss3d.png)
</center>
</p>
<p>
<big>[**nrays**](https://github.com/sebcrozet/nrays)</big> − a toy 3d and 4d
ray tracer in Rust. Obviously, it is used to test the ray-casting capabilities
of **ncollide**. In fact, most 3d illustration of this guide have been rendered
by **nrays**.
</p>
<p>
<center>
![Ray tracing demo](img/demo_ray_tracer.png)
</center>
</p>

-----


### Version 0.8.0
#### Added

* Added proximity queries, including persistant proximity detector and
  algorithm dispatcher.
* Added methods to set directly collision group membership/whitelist/blacklist.

#### Modified

* The last type parameter of the `BVTCostFn` trait (the user-defined data
  return by leaves) is now an associated type.
* The shape handles `Arc<Box<Repr<P, M>>>` are now wrapped into a structure
  with a more explicit name: `ShapeHandle<P, M>`.
* Renamed `Convex` to `ConvexHull`
* Swapped the first two arguments of `CompositeShape::map_transformed_part_at`.
* All fields of `Polyline` are now private. Added corresponding accessors.
