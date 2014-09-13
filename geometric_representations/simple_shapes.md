# Simple shapes

Those are the most basic geometric primitives supported by **ncollide**. They
are defined on the `geom` module. A geometric primitive does not have a
position in space. Instead, they are always positioned relative to some
absolute frame. Thus, one usually has to store a transformation matrix
separately from the shape itself.

## Ball
Mathematically speaking, the `Ball` structure describes a closed ball on the
_n_-dimensional euclidean space. In two dimensions this is a disk, and in three
dimensions a sphere, centered at the origin.


| Method | Description |
| --          | --       |
| `.radius()` | The radius of the ball. |

###### 2D and 3D example <span class="d3" onclick="window.open('../src/ball3d.rs')" ></span><span class="sp"></span><span class="d2" onclick="window.open('../src/ball2d.rs')"></span>

```rust
let ball = Ball::new(1.0);
assert!(ball.radius() == 1.0);
```

<center>
![2d ball](../img/ball2d.png)
![3d ball](../img/ball3d.png)
</center>


## Cuboid
The `Cuboid` structure describes a rectangle in two dimensions, or a cuboid in
three dimensions. A cuboid is defined by its _half extents_ − that is − its
half length along each coordinate axis.

| Method | Description |
| --          | --       |
| `.half_extents()` | The half extents of the cuboid. |

###### 2D example <span class="d2" onclick="window.open('../src/cuboid2d.rs')"></span>

```rust
let cuboid = Cuboid::new(Vec2::new(2.0, 1.0));

assert!(cuboid.half_extents().x == 2.0);
assert!(cuboid.half_extents().y == 1.0);
```

###### 3D example <span class="d3" onclick="window.open('../src/cuboid3d.rs')"></span>

```rust
let cuboid = Cuboid::new(Vec3::new(2.0, 1.0, 3.0));

assert!(cuboid.half_extents().x == 2.0);
assert!(cuboid.half_extents().y == 1.0);
assert!(cuboid.half_extents().z == 3.0);
```

<center>
![2d cuboid](../img/cuboid2d.png)
![3d cuboid](../img/cuboid3d.png)
</center>


## Cylinder
The `Cylinder` structure describes a rectangle in two dimensions (use `Cuboid`
instead), or a cylinder in three dimensions. Its principal axis is the
positive $$\bf y$$ axis.


| Method | Description |
| --       | --       |
| `.half_height()` | The half height of the cylinder. |
| `.radius()` | The radius of the cylinder basis. |

###### 3D example<span class="d3" onclick="window.open('../src/cylinder3d.rs')"></span>
```rust
let cylinder = Cylinder::new(0.5, 1.0);

assert!(cylinder.half_height() == 0.5);
assert!(cylinder.radius() == 1.0);
```

<center>
![cylinder](../img/cylinder3d.png)
</center>


## Cone
The `Cone` structure describes an isosceles triangle in two dimensions, or a
cone of revolution in tree dimensions. A cone is defined by the _radius_ of its
basis and its _half height_ − the half distance between the basis and the apex.
Its principal axis is the positive $$\bf y$$ axis.

| Method | Description |
| --          | --       |
| `.half_height()` | The half height of the cone. |
| `.radius()` | The radius of the cone basis. |

###### 3D example<span class="d3" onclick="window.open('../src/cone3d.rs')"></span>

```rust
let cone = Cone::new(0.5, 0.75);

assert!(cone.half_height() == 0.5);
assert!(cone.radius() == 0.75);
```

<center>
![cone](../img/cone3d.png)
</center>

## Capsule
The `Capsule` structure describes the Minkowski sum of a segment and a ball. In
other words, this is a cylinder with its flat extremities replaced by balls. A
capsule is defined by its _half height_ and the _radius_ of its extremities.
Its principal axis is the positive $$\bf y$$ axis.

| Method | Description |
| --          | --       |
| `.half_height()` | The half height of the capsule. |
| `.radius()` | The radius of the capsule extremities. |

###### 2D and 3D example <span class="d3" onclick="window.open('../src/capsule3d.rs')"></span><span class="sp"></span><span class="d2" onclick="window.open('../src/capsule2d.rs')"></span>
```rust
let capsule = Capsule::new(0.5, 0.75);

assert!(capsule.half_height() == 0.5);
assert!(capsule.radius() == 0.75);
```

<center>
![2d capsule](../img/capsule2d.png) 
![3d capsule](../img/capsule3d.png)
</center>

## Convex
The `Convex` structure describes a convex polyhedra. Remember that an object
is said to be convex if it is not self-crossing, and if it contains any segment
joining two of its points:

<center>
![Convex, concave, crossed](../img/convex_concave_crossing.svg)
</center>

There are two ways to create a `Convex` shape. Using the usual constructor
`::new(...)` will automatically compute the convex hull of the given array of
points.  However, if you already have a convex set of point, you may skip the
automatic convex hull computation using the unsafe constructor
`::new_with_convex_mesh(...)`. It is unsafe because the convexity of the
provided mesh is not checked.

| Method | Description  |
| --        | --           |
| `.mesh()` | The convex mesh. |
| `.pts()`  | The points of the convex mesh. |
| `.unwrap()` | Moves the contained `TriMesh` or `Polyline` out of the `Convex` structure. |

###### 2D example<span class="d2" onclick="window.open('../src/convex2d.rs')"></span>
```rust
let points = [
    Vec2::new(-1.0, 1.0), Vec2::new(-0.5, -0.5),
    Vec2::new(0.0, 0.5),  Vec2::new(0.5, -0.5),
    Vec2::new(1.0, 1.0)
];

let convex = Convex::new(points.as_slice());

assert!(convex.pts().len() == 4);
```
<center>
![2d convex](../img/convex2d.png)
</center>

###### 3D example<span class="d3" onclick="window.open('../src/convex3d.rs')"></span>
```rust
let points = [
    Vec3::new(0.0, 0.0, 1.0),
    Vec3::new(0.0, 0.0, -1.0),
    Vec3::new(0.0, 1.0, 0.0),
    Vec3::new(0.0, -1.0, 0.0),
    Vec3::new(1.0, 0.0, 0.0),
    Vec3::new(-1.0, 0.0, 0.0),
    Vec3::new(0.0, 0.0, 0.0)
];

let convex = Convex::new(points);

assert!(convex.pts().len() == 6);
```
<center>
![3d convex](../img/convex3d.png)
</center>

## Plane
The `Plane` structure describes a solid closed half-space. A plane is defined
by its _normal_. Every point that has a negative or zero dot product with the
plane normal is considered _inside_ of the plane. Other points are considered
_outside_ of the plane.

| Method   | Description  |
| --          | --        |
| `.normal()` | The normal of the plane. |

###### 2D example<span class="d2" onclick="window.open('../src/plane2d.rs')"></span>
```rust
let plane = Plane::new(Vec2::new(0.0, 1.0));

assert!(plane.normal().x == 0.0);
assert!(plane.normal().y == 1.0);
```

###### 3D example<span class="d3" onclick="window.open('../src/plane3d.rs')"></span>
```rust
let plane = Plane::new(Vec3::new(0.0, 1.0, 0.0));

assert!(plane.normal().x == 0.0);
assert!(plane.normal().y == 1.0);
assert!(plane.normal().z == 0.0);
```

## Mesh
The `Mesh` structure describes a polyline in two dimensions, or a triangle mesh
in three dimensions. A mesh is defined by an array of vertices and an array of
indices. Each segment (resp. triangle) in 2d (resp. 3d) is identified by two
(resp. three) indices. It is also possible to provide one normal and one
texture coordinate per vertex. Those are not used for the contact determination
but are useful for ray-casting. Internally, collision detection is accelerated
using an AABB tree.

| Method | Description |
| --          | --       |
| `.vertices()` | The vertex buffer. |
| `.indices()` | The index  buffer.  |
| `.normals()` | The normal buffer. |
| `.uvs()` | The texture coordinates buffer. |
| `.bounding_volumes()` | The bounding volume of each primitive (segment or triangle). |
| `.bvt()` | The space-partitioning acceleration structure used by the mesh. |

###### 2D example<span class="d2" onclick="window.open('../src/mesh2d.rs')"></span>

```rust
let points = vec!(
    Vec2::new(0.0, 1.0),  Vec2::new(-1.0, -1.0),
    Vec2::new(0.0, -0.5), Vec2::new(1.0, -1.0)
    );

let indices = vec!(0u, 1,
                   1,  2,
                   2,  3,
                   3,  1);

let mesh = Mesh::new(Arc::new(points), Arc::new(indices), None, None);

assert!(mesh.vertices().len() == 4);
```

<center>
![2d mesh](../img/mesh2d.png)
</center>


###### 3D example<span class="d3" onclick="window.open('../src/mesh3d.rs')"></span>

```rust
let points = vec!(
    Vec3::new(0.0, 1.0, 0.0),   Vec3::new(-1.0, -0.5, 0.0),
    Vec3::new(0.0, -0.5, -1.0), Vec3::new(1.0, -0.5, 0.0)
    );

let indices = vec!(0u, 1, 2,
                   0,  2, 3,
                   0,  3, 1);

let mesh = Mesh::new(Arc::new(points), Arc::new(indices), None, None);

assert!(mesh.vertices().len() == 4);
```

<center>
![3d mesh](../img/mesh3d.png)
</center>

