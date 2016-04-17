<center>
![ncollide](./img/logo_ncollide.svg)
</center>
<br/>
<center>
[![Crates.io Status](http://meritbadge.herokuapp.com/ncollide)](https://crates.io/crates/ncollide)
[![License (3-Clause BSD)](https://img.shields.io/badge/license-BSD%203--Clause-blue.svg?style=flat)](http://opensource.org/licenses/BSD-3-Clause)
[![Star](http://githubbadges.com/star.svg?user=sebcrozet&repo=ncollide&background=18bc9c&color=fff&style=flat)](http://github.com/sebcrozet/ncollide)

-----

<span class="h1 headline">2D and 3D collision detection library</span>
<div></div>
<span class="subheadline">For the [Rust](https://www.rust-lang.org) programming language.</span>
</center>

-----

## Demonstrations
1. [nphysics](http://nphysics.org) − a 2d and 3d physics engine available on
   [crates.io](http://crates.io) as the `nphysics2d` and `nphysics3d` crates.
   It completely relies on **ncollide** for contact points computation and
   proximity detection. It has at least one example per collision detection
   algorithm provided by **ncollide**.
<p>
<center>
<iframe width="560" height="315" src="http://www.youtube.com/embed/CANjXZ5rocI" frameborder="0" allowfullscreen></iframe>
</center>
</p>
2. [kiss3d](http://kiss3d.org) − a simplistic 3d graphics engine. It relies on
   mesh generation to render everything that is not triangular (sphere, cone,
   Bézier surfaces, etc.) In particular it has a
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
3. [nrays](https://github.com/sebcrozet/nrays) − a toy 3d and 4d ray tracer in
   Rust. Obviously, it is used to test the ray-casting capabilities of
   **ncollide**. In fact, most 3d illustration of this guide have been rendered
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
