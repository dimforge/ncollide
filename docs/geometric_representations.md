# Geometric representations
Different representations of geometric objects will lead to different
algorithms. Currently, **ncollide** relies a lot on convex shapes described by
a [support mapping](#support-mappings). However, working exclusively with
convex shapes is too restrictive so **ncollide** provides [composite
shapes](#composite-shapes) that allows the construction of a concave shape from
its convex parts.


Geometric primitives supported by **ncollide** are defined on the `shape`
module. They all share a common [dynamic
representation](#dynamic-shape-representation).  Note that all geometric
primitives have a predefined constant local frame equal to the identity matrix.
Thus, one usually has to store a transformation matrix separately from
the shape itself in order to reach any location and orientation.

# Support mappings

**ncollide** supports generic algorithms that work for any (possibly
user-defined) shape defined by a support map. The support map (argument) of a
shape $\mathcal{A}$ is a function that returns the point $\mathbf{p}$ that
maximises its dot product with a given direction $\mathbf{v}$. Such a point
point $s_{\mathcal{A}}(\mathbf{v})$ is called a _support point_:

$$
s_{\mathcal{A}}(\mathbf{v}) = \arg \max\limits_{\mathbf{p} \in \mathcal{A}} \left< \mathbf{p}, \mathbf{v} \right>
$$

 This can be seen as a function that returns a point of the support mapped
 shape which is _the furthest on the given direction_. Such a function is
 enough to describe completely a convex object. If several points are eligible
 to be support points for a given direction $\mathbf{v}$, any one of them can
 be returned (preferably a corner). The following shows support points for the
 shapes $\mathcal{A}, \mathcal{B}$ and $\mathcal{C}$, given two directions
 $\mathbf u$ and $\mathbf v$:

<center>
![Support function](../img/support_fun_simple.svg)
</center>

The support mapping function is exposed by the `SupportMap` trait.

| Method                            | Description |
|--                                 | --          |
| `.support_point(m, v)`            | Computes the support point (in the direction `v`) of the caller transformed by the transformation matrix `m`. |

Most basic geometric primitives like balls, cubes, cones, and even more complex
ones like a Minkowski Sum of convex shapes can be described by their support
mappings. This allows a useful level of genericity for several geometric
queries on **ncollide**. Moreover, allogithms based on support algorithms can
usually general enough to be able to operate on shapes embedded on a space of
finite dimension higher than 3.

### Ball
Mathematically speaking, the `Ball` structure describes a closed ball on the
_n_-dimensional euclidean space. In two dimensions this is a disk, and in three
dimensions a sphere, both centered at the origin.


| Method | Description |
| --          | --       |
| `.radius()` | The radius of the ball. |

<ul class="nav nav-tabs">
  <li class="active"><a id="tab_nav_link" data-toggle="tab" href="#ball">Example</a></li>
  <div class="btn-primary" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/ball.rs')"></div>
</ul>

<div class="tab-content" markdown="1">
  <div id="ball" class="tab-pane in active">
```rust
let ball = Ball::new(1.0f32);
assert!(ball.radius() == 1.0);
```
  </div>
</div>

<center>
![2D ball](../img/ball2d.png)
![3D ball](../img/ball3d.png)
</center>

-----------

### Cuboid
The `Cuboid` structure describes a rectangle in two dimensions or a cuboid in
three dimensions. A cuboid is defined by its _half extents_, i.e., its
half length along each coordinate axis.

| Method | Description |
| --          | --       |
| `.half_extents()` | The half extents of the cuboid. |

<ul class="nav nav-tabs">
  <li class="active"><a id="tab_nav_link" data-toggle="tab" href="#cuboid_2D">2D example</a></li>
  <li><a id="tab_nav_link" data-toggle="tab" href="#cuboid_3D">3D example</a></li>
  <div class="d3" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/cuboid3d.rs')"></div>
  <div class="sp"></div>
  <div class="d2" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/cuboid2d.rs')"></div>
</ul>

<div class="tab-content" markdown="1">
  <div id="cuboid_2D" class="tab-pane in active">
```rust
let cuboid = Cuboid::new(Vector2::new(2.0f32, 1.0));

assert!(cuboid.half_extents().x == 2.0);
assert!(cuboid.half_extents().y == 1.0);
```
  </div>
  <div id="cuboid_3D" class="tab-pane">
```rust
let cuboid = Cuboid::new(Vector3::new(2.0f32, 1.0, 3.0));

assert!(cuboid.half_extents().x == 2.0);
assert!(cuboid.half_extents().y == 1.0);
assert!(cuboid.half_extents().z == 3.0);
```
  </div>
</div>

<center>
![2D cuboid](../img/cuboid2d.png)
![3D cuboid](../img/cuboid3d.png)
</center>


-----------

### Cylinder
The `Cylinder` structure describes a rectangle in two dimensions or a cylinder
in three dimensions. Its principal axis is the $\mathbf{y}$ axis.


| Method | Description |
| --       | --       |
| `.half_height()` | The half height of the cylinder. |
| `.radius()` | The radius of the cylinder basis. |

<ul class="nav nav-tabs">
  <li class="active"><a id="tab_nav_link" data-toggle="tab" href="#cylinder">Example</a></li>
  <div class="btn-primary" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/cylinder.rs')"></div>
</ul>

<div class="tab-content" markdown="1">
  <div id="cylinder" class="tab-pane in active">
```rust
let cylinder = Cylinder::new(0.5f32, 1.0);

assert!(cylinder.half_height() == 0.5);
assert!(cylinder.radius() == 1.0);
```
  </div>
</div>

<center>
![cylinder](../img/cylinder3d.png)
</center>


-----------

### Cone
The `Cone` structure describes an isosceles triangle in two dimensions or a
cone of revolution in three dimensions. A cone is defined by the _radius_ of its
basis and its _half height_ − the half distance between the basis and the apex.
It points upward, its principal axis is the $\mathbf{y}$ axis, and its apex has
coordinates $(0, \text{cone.half_height()}, 0)$.

| Method | Description |
| --          | --       |
| `.half_height()` | The half height of the cone. |
| `.radius()` | The radius of the cone basis. |

<ul class="nav nav-tabs">
  <li class="active"><a id="tab_nav_link" data-toggle="tab" href="#cone">Example</a></li>
  <div class="btn-primary" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/cone.rs')"></div>
</ul>

<div class="tab-content" markdown="1">
  <div id="cone" class="tab-pane in active">
```rust
let cone = Cone::new(0.5f32, 0.75);

assert!(cone.half_height() == 0.5);
assert!(cone.radius() == 0.75);
```
</div>
</div>

<center>
![2D cone](../img/cone2d.png)
![3D cone](../img/cone3d.png)
</center>

-----------

### Capsule
The `Capsule` structure describes the Minkowski sum of a segment and a ball. In
other words, this is a cylinder with extremities replaced by half-balls. A
capsule is defined by the _half height_ of its cylindrical part and the
_radius_ of its extremities.  It is centered at the origin and its principal
axis is the $\mathbf{y}$ axis.

| Method | Description |
| --          | --       |
| `.half_height()` | The half height of the capsule cylindrical part. |
| `.radius()` | The radius of the capsule extremities. |

<ul class="nav nav-tabs">
  <li class="active"><a id="tab_nav_link" data-toggle="tab" href="#capsule">Example</a></li>
  <div class="btn-primary" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/capsule.rs')"></div>
</ul>

<div class="tab-content" markdown="1">
  <div id="capsule" class="tab-pane in active">
```rust
let capsule = Capsule::new(0.5f32, 0.75);

assert!(capsule.half_height() == 0.5);
assert!(capsule.radius() == 0.75);
```
</div>
</div>

<center>
![2D capsule](../img/capsule2d.png)
![3D capsule](../img/capsule3d.png)
</center>

-----------

### Convex hull
The `ConvexHull` structure represents the smallest convex envelope of a set of
point. Remember that an object is said to be convex if it is not self-crossing,
and if it contains any segment joining two of its points:

<center>
![convex, concave, crossed](../img/convex_concave_crossing.svg)
</center>

The `ConvexHull` shape is created from a set of points. Note that it does not
compute explicitly the convex hull of the points so its creation takes
$\mathcal{O}(1)$ time.

| Method | Description  |
| --        | --           |
| `.points()`  | The points used to create the `ConvexHull` shape. |

<ul class="nav nav-tabs">
  <li class="active"><a id="tab_nav_link" data-toggle="tab" href="#convex_2D">2D example</a></li>
  <li><a id="tab_nav_link" data-toggle="tab" href="#convex_3D">3D example</a></li>

  <div class="d3" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/convex3d.rs')"></div>
  <div class="sp"></div>
  <div class="d2" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/convex2d.rs')"></div>
</ul>

<div class="tab-content" markdown="1">
  <div id="convex_2D" class="tab-pane in active">
```rust
let points = vec![
    Point2::new(-1.0f32, 1.0), Point2::new(-0.5, -0.5),
    Point2::new(0.0, 0.5),     Point2::new(0.5, -0.5),
    Point2::new(1.0, 1.0)
];

let convex = ConvexHull::new(points);

// ConvexHull does not compute explicitly the convex hull (which has 4 vertices)!
assert!(convex.points().len() == 5);
```
  </div>
  <div id="convex_3D" class="tab-pane">
```rust
let points = vec![
    Point3::new(0.0f32, 0.0, 1.0),
    Point3::new(0.0, 0.0, -1.0),
    Point3::new(0.0, 1.0, 0.0),
    Point3::new(0.0, -1.0, 0.0),
    Point3::new(1.0, 0.0, 0.0),
    Point3::new(-1.0, 0.0, 0.0),
    Point3::new(0.0, 0.0, 0.0)
];

let convex = ConvexHull::new(points);

// ConvexHull does not compute explicitly the convex hull (which has 6 vertices)!
assert!(convex.points().len() == 7);
```
  </div>
</div>

<center>
![2D convex](../img/convex2d.png)
![3D convex](../img/convex3d.png)
</center>

-----------

### Reflection
The `Reflection` structure describes the reflection of a shape $\mathcal{A}$
with regard to the origin:

$$
-\mathcal{A} = \left\{ -\mathbf{a} \mid{} \mathbf{a} \in \mathcal{A} \right\}
$$

The reflected shape and the reflection itself are lifetime-bound so it cannot
be used to create a [shape handle](#the-shape-handle).

| Method | Description |
| --       | --        |
| `.shape()` | The shape affected by the reflection. |

<ul class="nav nav-tabs">
  <li class="active"><a id="tab_nav_link" data-toggle="tab" href="#reflection">Example</a></li>
  <div class="btn-primary" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/reflection.rs')"></div>
</ul>

<div class="tab-content" markdown="1">
  <div id="reflection" class="tab-pane in active">
```rust
let cone = Cone::new(0.5f32, 0.75);

// Build the reflection.
let _ = Reflection::new(&cone);
```
</div>
</div>

<center>
![reflected 2D cone](../img/refl2d.png)
![reflected 3D cone](../img/refl3d.png)
</center>

-----------

### Minkowski sum
The `MinkowskiSum` structure describes the Minkoswki sum of two shapes
implementing the `SupportMap` trait. If $\mathcal{A}$ and $\mathcal{B}$ are
two (possibly infinite) sets of points, their Minkowski sum is given by the set:

$$
\mathcal{A} \oplus \mathcal{B} = \left\{ \mathbf{a} + \mathbf{b} \mid{} \mathbf{a} \in \mathcal{A}, \mathbf{b} \in \mathcal{B} \right\}
$$

In other words, this is the union of all points of one shape successively
translated by each point of the other one. This is extremely useful for
discrete and continuous collision detection. Note that `MinkowskiSum` does not
compute the sum explicitly so its construction is $\mathcal{O}(1)$. It is
lifetime-bound to the shapes involved in the sum so it cannot be used to create
a [shape handle](#dynamic-shape-representation).

| Method | Description |
| --       | --        |
| `.m1()` | The local transformation of the **first** shape involved in the sum. |
| `.m2()` | The local transformation of the **second** shape involved in the sum. |
| `.g1()` | The **first** shape involved in the sum. |
| `.g2()` | The **second** shape involved in the sum. |

<ul class="nav nav-tabs">
  <li class="active"><a id="tab_nav_link" data-toggle="tab" href="#minkowski_sum_2D">2D example</a></li>
  <li><a id="tab_nav_link" data-toggle="tab" href="#minkowski_sum_3D">3D example</a></li>

  <div class="d3" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/minkowski_sum3d.rs')"></div>
  <div class="sp"></div>
  <div class="d2" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/minkowski_sum2d.rs')"></div>
</ul>

<div class="tab-content" markdown="1">
  <div id="minkowski_sum_2D" class="tab-pane in active">
```rust
let cylinder = Cylinder::new(0.5f32, 0.75);
let cone     = Cone::new(0.75f32, 0.75);

let delta_cylinder = na::one::<Isometry2<f32>>(); // Identity matrix.
let delta_cone     = na::one::<Isometry2<f32>>(); // Identity matrix.

let _ = MinkowskiSum::new(&delta_cylinder, &cylinder, &delta_cone, &cone);
```
  </div>
  <div id="minkowski_sum_3D" class="tab-pane">
```rust
let cylinder = Cylinder::new(0.5f32, 0.75);
let cone     = Cone::new(0.75f32, 0.75);

let delta_cylinder = na::one::<Isometry3<f32>>(); // Identity matrix.
let delta_cone     = na::one::<Isometry3<f32>>(); // Identity matrix.

let _ = MinkowskiSum::new(&delta_cylinder, &cylinder, &delta_cone, &cone);
```
</div>
</div>

<center>
![2D cylinder + cone](../img/msum2d.png)
![3D cylinder + cone](../img/msum3d.png)
</center>

#### Configuration Space Obstacle construction example <div class="d3" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/configuration_space_obstacle3d.rs')"></div><div class="sp"></div><div class="d2" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/configuration_space_obstacle2d.rs')"></div>
The Configuration Space Obstacle (CSO) is the same as the Minkowski sum of the
first shape with the reflection of the second one:

$$
\mathcal{A} \ominus \mathcal{B} = \mathcal{A} \oplus -\mathcal{B} = \left\{ \mathbf{a} - \mathbf{b} \mid{} \mathbf{a} \in \mathcal{A}, \mathbf{b} \in \mathcal{B} \right\}
$$

This concept is extensively used, e.g., in robotics and interference detection
to compute all the impossible positions of the center of mass of an object that
evolves in a complex environment. This is obviously **not** a
commutative operator. It can be constructed using the `MinkowskiSum` and
`Reflection` shapes:

```rust
let cylinder   = Cylinder::new(0.5f32, 0.75);
let cone       = Cone::new(0.75f32, 0.75);
let reflection = Reflection::new(&cone); // Take the reflection of the cone.

let delta_cylinder = na::one::<Isometry3<f32>>(); // Identity matrix.
let delta_cone     = na::one::<Isometry3<f32>>(); // Identity matrix.

// Build the Configuration Space Obstacle.
let _ = MinkowskiSum::new(&delta_cylinder, &cylinder, &delta_cone, &reflection);
```

<center>
![2D cylinder - cone](../img/cso2d.png)
![3D cylinder - cone](../img/cso3d.png)
</center>

# Composite shapes

**ncollide** supports shapes that are defined as aggregations of others. Every
composite shape must implement the `CompositeShape` trait which defines methods
for accessing their individual parts using indices. The composite is assumed to
be immutable, i.e., an index must always map to a shape and local
transformation that both remain constant over time. However, the actual range
of index values is implementation-defined and does not even have to be
contiguous nor finite.

| Method | Description |
| --          | --        |
| `.map_part_at(i, f)` | Applies the closure `f` to the `i`-th part and its local transformation matrix. |
| `.map_transformed_part_at(i, m, f)` | Applies the closure `f` to the `i`-th part and its local transformation matrix with `m` appended to it. |
| `.aabb_at(i)` | The `AABB` of the `i`-th part of the composite shape. |
| `.bvt()` | The space-partitioning acceleration structure used by the composite shape. |

The requirement to use a `BVT` for space-partitioning is extremely restrictive
and will be replaced by a more flexible system in the future. Currently, three
composite shapes are available on **ncollide**. The `Compound` describes the
union of any shape supported by **ncollide**. The `TriMesh` and the `Polyline`
are dedicated to assemblies of triangles and segments.

### Compound
The `Compound` structure is the main way of describing concave shapes from
convex ones. It is a set of any [shape handle](#dynamic-shape-representation).

| Method | Description |
| --          | --        |
| `.shapes()` | The shapes composing the compound. |
| `.bounding_volumes()` | The `AABB` of the shapes composing the compound. |
| `.bvt()` | The space-partitioning acceleration structure used by the compound. |


Two steps are necessary to create a `Compound`:

1. Initialize a vector of shape handles with their positions and orientations
   relative to the origin.
2. Call `Compound::new` with this vector.

<ul class="nav nav-tabs">
  <li class="active"><a id="tab_nav_link" data-toggle="tab" href="#compound_2D">2D example</a></li>
  <li><a id="tab_nav_link" data-toggle="tab" href="#compound_3D">3D example</a></li>

  <div class="d3" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/compound3d.rs')"></div>
  <div class="sp"></div>
  <div class="d2" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/compound2d.rs')"></div>
</ul>

<div class="tab-content" markdown="1">
  <div id="compound_2D" class="tab-pane in active">
```rust
// Delta transformation matrices.
let delta1 = Isometry2::new(Vector2::new(0.0f32, -1.5), na::zero());
let delta2 = Isometry2::new(Vector2::new(-1.5f32, 0.0), na::zero());
let delta3 = Isometry2::new(Vector2::new(1.5f32,  0.0), na::zero());

// 1) Initialize the shape list.
let mut shapes = Vec::new();
let horizontal_box = ShapeHandle::new(Cuboid::new(Vector2::new(1.5f32,  0.25)));
let vertical_box   = ShapeHandle::new(Cuboid::new(Vector2::new(0.25f32, 1.5)));

shapes.push((delta1, horizontal_box));
shapes.push((delta2, vertical_box.clone()));
shapes.push((delta3, vertical_box));

// 2) Create the compound shape.
let compound = Compound::new(shapes);

assert!(compound.shapes().len() == 3)
```
  </div>
  <div id="compound_3D" class="tab-pane">
```rust
// Delta transformation matrices.
let delta1 = Isometry3::new(Vector3::new(0.0f32, -1.5, 0.0), na::zero());
let delta2 = Isometry3::new(Vector3::new(-1.5f32, 0.0, 0.0), na::zero());
let delta3 = Isometry3::new(Vector3::new(1.5f32, 0.0,  0.0), na::zero());

// 1) Initialize the shape list.
let mut shapes = Vec::new();
let horizontal_box = ShapeHandle::new(Cuboid::new(Vector3::new(1.5f32,  0.25, 0.25)));
let vertical_box   = ShapeHandle::new(Cuboid::new(Vector3::new(0.25f32, 1.5, 0.25)));

shapes.push((delta1, horizontal_box));
shapes.push((delta2, vertical_box.clone()));
shapes.push((delta3, vertical_box));

// 2) Create the compound shape.
let compound = Compound::new(shapes);

assert!(compound.shapes().len() == 3)
```
  </div>
</div>


<center>
![2D compound](../img/compound2d.png)
![3D compound](../img/compound3d.png)
</center>

-----------

### Polyline and TriMesh
The `Polyline` and `TriMesh` structures describe a set of segments and a mesh
of triangles. They are constructed from shared arrays of vertices and indices.
Each segment (resp. triangle) is identified by two (resp. three) indices. It is
also possible to provide one normal and one texture coordinate per vertex;
those are not used for contact determination but are useful for, e.g.,
ray-tracing applications.

| Method | Description |
| --          | --       |
| `.vertices()` | The vertex buffer. |
| `.indices()` | The index  buffer.  |
| `.normals()` | The normal buffer. |
| `.uvs()` | The texture coordinates buffer. |
| `.bounding_volumes()` | The bounding volume of each primitive (segment or triangle). |
| `.bvt()` | The space-partitioning acceleration structure used by the mesh. |


<ul class="nav nav-tabs">
  <li class="active"><a id="tab_nav_link" data-toggle="tab" href="#mesh_2D">2D example</a></li>
  <li><a id="tab_nav_link" data-toggle="tab" href="#mesh_3D">3D example</a></li>

  <div class="d3" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/mesh3d.rs')"></div>
  <div class="sp"></div>
  <div class="d2" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/mesh2d.rs')"></div>
</ul>

<div class="tab-content" markdown="1">
  <div id="mesh_2D" class="tab-pane in active">
```rust
let points = vec!(
    Point2::new(0.0, 1.0),  Point2::new(-1.0, -1.0),
    Point2::new(0.0, -0.5), Point2::new(1.0, -1.0));

let indices = vec!(Point2::new(0usize, 1),
                   Point2::new(1, 2),
                   Point2::new(2, 3),
                   Point2::new(3, 1));

// Build the polyline.
let polyline = Polyline::new(Arc::new(points), Arc::new(indices), None, None);

assert!(polyline.vertices().len() == 4);
```
  </div>
  <div id="mesh_3D" class="tab-pane">
```rust
let points = vec!(
    Point3::new(0.0, 1.0, 0.0), Point3::new(-1.0, -0.5, 0.0),
    Point3::new(0.0, -0.5, -1.0), Point3::new(1.0, -0.5, 0.0));

let indices = vec!(Point3::new(0usize, 1, 2),
                   Point3::new(0, 2, 3),
                   Point3::new(0, 3, 1));

// Build the mesh.
let mesh = TriMesh::new(Arc::new(points), Arc::new(indices), None, None);

assert!(mesh.vertices().len() == 4);
```
</div>
</div>

<center>
![2D mesh](../img/mesh2d.png)
![3D mesh](../img/mesh3d.png)
</center>

# Other shapes
Some shapes do not fall into any of the general categories described above.

### Plane
The `Plane` structure describes a solid closed half-space. The border of a
plane contains the origin and is characterized by its _normal_. Every point
that has a negative or null dot product with the plane normal is considered
_inside_ of it. Other points are _outside_ of the plane.

| Method   | Description  |
| --          | --        |
| `.normal()` | The normal of the plane. |

<ul class="nav nav-tabs">
  <li class="active"><a id="tab_nav_link" data-toggle="tab" href="#plane_2D">2D example</a></li>
  <li><a id="tab_nav_link" data-toggle="tab" href="#plane_3D">3D example</a></li>

  <div class="d3" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/plane3d.rs')"></div>
  <div class="sp"></div>
  <div class="d2" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/plane2d.rs')"></div>
</ul>

<div class="tab-content" markdown="1">
  <div id="plane_2D" class="tab-pane in active">
```rust
let plane = Plane::new(Vector2::new(0.0f32, 1.0));

assert!(plane.normal().x == 0.0);
assert!(plane.normal().y == 1.0);
```
  </div>
  <div id="plane_3D" class="tab-pane">
```rust
let plane = Plane::new(Vector3::new(0.0f32, 1.0, 0.0));

assert!(plane.normal().x == 0.0);
assert!(plane.normal().y == 1.0);
assert!(plane.normal().z == 0.0);
```
  </div>
</div>

# Dynamic shape representation

In order to select the right algorithms for geometric queries on specific
shapes, `ncollide` has to be able to distinguish at runtime different shapes
from their types and they capabilities. As described by [another
chapter](../geometric_queries) of this guide, there are two kinds of geometric
queries: those that operate on a [single
shape](../geometric_queries/#single-shape-queries) and those that operate on
[two shapes](../geometric_queries/#pairwise-queries) simultaneously.  In the
first case, runtime algorithm selection is performed with the help of traits
which can be easily implemented for user-defined shapes and exposed at runtime
by a `Shape` trait-object. On the other hand, queries involving two shapes
require more complex dispatch mechanisms that are not yet customizable by the
user.

### The shape handle
Elements to inspect shape representation and capabilities are provided for each
shape by implementing the `Shape` trait. The `ShapeHandle` structure is nothing
more than a `Shape` trait-object wrapped into an `Arc`.

| Method                    | Description |
|--                         | --          |
| `.aabb(m: &M)`            | The AABB of the shape transformed by `m`.            |
| `.bounding_sphere(m: &M)` | The bounding sphere of the shape transformed by `M`. |
| `.as_ray_cast()`          | Converts `self` to a `RayCast` trait-object.         |
| `.as_point_query()`       | Converts `self` to a `PointQuery` trait-object.      |
| `.as_support_map()`       | Converts `self` to a `SupportMap` trait-object.      |
| `.as_composite_shape()`   | Converts `self` to a `CompositeShape` trait-object.   |
| `.is_support_map()`       | Returns `true` if this shape has a support-mapping.  |
| `.is_composite_shape()`   | Returns `true` if this shape is a composite shape.   |

All the conversion methods have a default implementation returning `None`.
This allows you to only partially implement the trait if some of those features
are of no interest to you or even not applicable for your specific shape. For
example it is extremely rare to implement both `.as_composite_shape()` and
`.as_support_map()` as algorithms applicable to the latter will almost always
be more efficient than for the former.

Methods related to bounding volumes have default implementations as well
except for AABB construction. AABBs are widely used throughout **ncollide** so
their tightness is critical for good performances. Other bounding volumes are
less used so by default they are deduced from the AABB itself. Those default
implementations will unfortunately only result in very loose bounding volumes
so it is advised to provide your own to ensure optimal performances.

### Custom support mapping <div class="btn-primary" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/custom_support_map.rs')"></div>
In this section we detail an example to define your own support-mapped shape
suitable to be wrapped into a `ShapeHandle`. Too keep the maths simple, we will
define a 2-dimensional ellipse centered at the origin with radii $a$ and $b$:

<center>
![crossed cuboids](../img/ellipse.svg)
</center>

```rust
struct Ellipse {
    a: f32, // The first radius.
    b: f32  // The second radius.
}
```

We first have to define its support mapping.  Let $\mathcal{F}$ be an ellipse.
The implicit equation of the border of an ellipse centered at the origin is
given by $f(x, y) = \left(\frac{x}{a}\right)^2 + \left(\frac{y}{b}\right)^2 - 1
= 0$. Therefore its gradient is simply $\nabla{}f(x, y) = \left[
\frac{2x}{a^2}, \frac{2y}{b^2} \right]^\intercal$.  Then, for some direction
$\mathbf{v} = [ x_v, y_v ]^\intercal \in \mathbb{R}^2$, convex analysis tells
us that the point $ [ x^*, y^* ]^\intercal \in \mathcal{F}$ that maximizes or
minimizes the dot product with $\mathbf{v}$ is such that $v \times
\nabla{}f(x^*, y^*) = \frac{x_vy^*}{b^2} - \frac{y_vx^*}{a^2} = 0$. Thus, we
have to solve the quadratic algebraic system:

$$
\left\{
\begin{array}{lcl}
    \frac{x_v y^*}{b^2} - \frac{y_vx^*}{a^2} & = & 0\\
    \left(\frac{x^*}{a}\right)^2 + \left(\frac{y^*}{b}\right)^2 - 1 & = & 0
\end{array}
\right.
$$

The solutions of this system are $[ x^*, y^* ]^\intercal = \pm{}\left[ \frac{x_v a^2}{\sqrt{x_v^2 a^2 + y_v^2 b^2}}, \frac{y_vb^2}{\sqrt{x_v^2 a^2 + y_v^2 b^2}} \right]^\intercal$.
Among those two solutions, the maximizer is the first one (with the leading $+$ sign).
Now, we only have to code this into a `SupportMap` implementation for our
`Ellipse`.

```rust
impl SupportMap<Point2<f32>, Isometry2<f32>> for Ellipse {
    fn support_point(&self, transform: &Isometry2<f32>, dir: &Vector2<f32>) -> Point2<f32> {
        // Bring `dir` into the ellipse's local frame.
        let local_dir = transform.inverse_transform_vector(dir);

        // Compute the denominator.
        let denom = f32::sqrt(local_dir.x * local_dir.x * self.a * self.a +
                              local_dir.y * local_dir.y * self.b * self.b);

        // Compute the support point into the ellipse's local frame.
        let local_support_point = Point2::new(self.a * self.a * local_dir.x / denom,
                                              self.b * self.b * local_dir.y / denom);

        // Return the support point transformed back into the global frame.
        *transform * local_support_point
    }
}
```

With this, we will be able to pass an instance of `Ellipse` to many geometric
queries from the `query::algorithms` module and all the functions with names
that contain `support_map` on the other submodules of the `query` module.
If we want to benefit from a higher-level interface based on the dynamic
dispatch mechanism of **ncollide**, we have to implement the `Shape` trait
providing at least an AABB and a conversion to the `SupportMap` trait-object:

```rust
impl Shape<Point2<f32>, Isometry2<f32>> for Ellipse {
    fn aabb(&self, m: &Isometry2<f32>) -> AABB2<f32> {
        // Generic method to compute the aabb of a support-mapped shape.
        bounding_volume::support_map_aabb(m, self)
    }

    fn as_support_map(&self) -> Option<&SupportMap2<f32>> {
        Some(self)
    }
}
```

That's it! You will now be able to pass an ellipse to various functions of
the `query` module, and even wrap it on a `ShapeHandle` to use it in a
[collision object](../collision_detection_pipeline#collision-objects) for
complex interactions with a `CollisionWorld` using the [collision detection
pipeline](../collision_detection_pipeline). The following is an example of some
pairwise [geometric queries](../geometric_queries/#pairwise-queries) involving
our own ellipse:

```rust
let ellipse = Ellipse { a: 2.0f32, b: 1.0 };
let cuboid  = Cuboid::new(Vector2::new(1.0, 1.0));

let ellipse_pos = na::one();
let cuboid_pos  = Isometry2::new(Vector2::new(4.0, 0.0), na::zero());

let dist = query::distance(&ellipse_pos, &ellipse, &cuboid_pos, &cuboid);
let prox = query::proximity(&ellipse_pos, &ellipse, &cuboid_pos, &cuboid, 0.0);
let ctct = query::contact(&ellipse_pos, &ellipse, &cuboid_pos, &cuboid, 0.0);

assert!(na::approx_eq(&dist, &1.0));
assert_eq!(prox, Proximity::Disjoint);
assert!(ctct.is_none());
```

### Custom composite shape <div class="btn-primary" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/custom_composite_shape.rs')"></div>
In this section we detail an example to define your own composite shape
suitable to be wrapped into a `ShapeHandle`.  We will define a simple 2D
cross-shaped object with only two part:

1. A Cuboid of width 4 and height 2.
2. A Cuboid of width 2 and height 4.

Both will be centered at the point (1, 1):

<center>
![crossed cuboids](../img/crossed_cuboids.svg)
</center>

Because the two parts are very simple, we will generate them on-the-fly when
needed by the user. Only the acceleration structure will be precomputed.

```rust
struct CrossedCuboids {
    bvt: BVT<usize, AABB2<f32>>
}

impl CrossedCuboids {
    pub fn new() -> CrossedCuboids {
        // The shape indices paired with their corresponding AABBs.
        // Nedded to initialize the acceleration structure.
        let aabbs = vec! [
            (0, CrossedCuboids::generate_aabb(0)),
            (1, CrossedCuboids::generate_aabb(1))
        ];

        CrossedCuboids {
            bvt: BVT::new_balanced(aabbs)
        }
    }

    // Helper function to generate the AABB bounding the i-th cuboid.
    fn generate_aabb(i: usize) -> AABB2<f32> {
        if i == 0 {
            // The AABB for the horizontal cuboid.
            AABB2::new(Point2::new(-1.0, 0.0), Point2::new(3.0, 2.0))
        }
        else {
            // The AABB for the vertical cuboid.
            AABB2::new(Point2::new(0.0, -1.0), Point2::new(2.0, 3.0))
        }
    }

    // Helper function to generate the i-th cuboid.
    fn generate_cuboid(i: usize) -> Cuboid2<f32> {
        if i == 0 {
            // Create a 4x2 cuboid. Remember that we must provide the
            // half-lengths.
            Cuboid2::new(Vector2::new(2.0, 1.0))
        }
        else {
            // Create a 2x4 cuboid. Remember that we must provide the
            // half-lengths.
            Cuboid2::new(Vector2::new(1.0, 2.0))
        }
    }
}
```

Now we have to implement the `CompositeShape` trait.

```rust
impl CompositeShape<Point2<f32>, Isometry2<f32>> for CrossedCuboids {
    fn map_part_at(&self, i: usize, f: &mut FnMut(&Isometry2<f32>, &Shape2<f32>)) {
        // The translation needed to center the cuboid at the point (1, 1).
        let transform = Isometry2::new(Vector2::new(1.0, 1.0), na::zero());

        // Create the cuboid on-the-fly.
        let cuboid = CrossedCuboids::generate_cuboid(i);

        // Call the function.
        f(&transform, &cuboid)
    }

    fn map_transformed_part_at(&self,
                               i: usize,
                               m: &Isometry2<f32>,
                               f: &mut FnMut(&Isometry2<f32>, &Shape2<f32>)) {
        // Prepend the translation needed to center the cuboid at the point (1, 1).
        let transform = m * Translation2::new(1.0, 1.0);

        // Create the cuboid on-the-fly.
        let cuboid = CrossedCuboids::generate_cuboid(i);

        // Call the function.
        f(&transform, &cuboid)
    }

    fn aabb_at(&self, i: usize) -> AABB2<f32> {
        // Compute the i-th AABB.
        CrossedCuboids::generate_aabb(i)
    }

    fn bvt(&self) -> &BVT<usize, AABB2<f32>> {
        // Reference to the acceleration structure.
        &self.bvt
    }
}
```

Just like in the previous section, implementing the `CompositeShape` trait is
enough to use some pairwise geometric queries, e.g., those from the submodule
of `query` module with names that contain the word `composite`. For a more
complete integration, we have to implement the `Shape` trait providing at least
an AABB and a conversion to the `CompositeShape` trait-object:

```rust
impl Shape<Point2<f32>, Isometry2<f32>> for CrossedCuboids {
    fn aabb(&self, m: &Isometry2<f32>) -> AABB2<f32> {
        // This is far from an optimal AABB.
        AABB2::new(m.translation * Point2::new(-10.0, -10.0),
                   m.translation * Point2::new(10.0, 10.0))
    }

    fn as_composite_shape(&self) -> Option<&CompositeShape2<f32>> {
        Some(self)
    }
}
```

That's it! You will now be able to pass a cross-like shape to various
functions of the `query` module, and even wrap it on a `ShapeHandle` to use
it in a [collision object](../collision_detection_pipeline#collision-objects)
for complex interactions with a `CollisionWorld` using the [collision detection
pipeline](../collision_detection_pipeline). The following is an example of
some pairwise [geometric queries](../geometric_queries/#pairwise-queries)
involving our own composite shape:

```rust
let ellipse = Ellipse { a: 2.0f32, b: 1.0 };
let cuboid  = Cuboid::new(Vector2::new(1.0, 1.0));

let ellipse_pos = na::one();
let cuboid_pos  = Isometry2::new(Vector2::new(4.0, 0.0), na::zero());

let dist = query::distance(&ellipse_pos, &ellipse, &cuboid_pos, &cuboid);
let prox = query::proximity(&ellipse_pos, &ellipse, &cuboid_pos, &cuboid, 0.0);
let ctct = query::contact(&ellipse_pos, &ellipse, &cuboid_pos, &cuboid, 0.0);

assert!(relative_eq!(dist, 1.0, epsilon = 1.0e-6));
assert_eq!(prox, Proximity::Disjoint);
assert!(ctct.is_none());
```
