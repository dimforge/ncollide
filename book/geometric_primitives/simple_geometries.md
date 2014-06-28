# Simple geometries

## The Ball
Mathematically speaking, the `Ball` structure describes a closed ball on the
_n_-dimensional euclidean space. In two dimensions, this is a disk, and in
three dimensions, this is a sphere, centered at the origin.


| Description | Accessors | Value |
| --          | --       | --    |
| The radius of the ball | `b.radius()` | User-defined with `Ball::new` |

###### 2D and 3D example:

```rust
let ball = Ball::new(1.0);
assert!(ball.radius() == 1.0);
```

![fixme](ball2d) ![fixme](ball3d)


## The Cuboid
The `Cuboid` structure describes a rectangle in two dimensions, or a cuboid in
three dimensions. A cuboid is defined by its _half extents_ − that is − its
half length along each coordinate axis. For performance and accuracy reasons,
each cuboid also has an _internal margin_. This means that the actual geometry
used for collision detection is a cuboid with rounded corners with a radius
equal to the margin.

| Description | Accessors | Value |
| --          | --        | --    |
| The half extents of the cuboid | `c.half_extents()` | User-defined by `Cuboid::new` |
| The internal margin of the cuboid | `c.margin()` | `0.04` or user-defined by `Cuboid::new_with_margin` |

###### 3D example:
```rust
let cuboid = Cuboid::new(Vec3::new(1.0, 2.0, 3.0));
assert!(cuboid.margin() == 0.04); // default margin.

```

 ![fixme](cuboid3d)

###### 2D example:
```rust
let cuboid = Cuboid::new_with_margin(Vec2::new(1.0, 2.0), 0.2);
assert!(cuboid.margin() == 0.2); // user-defined margin
```

![fixme](cuboid2d)


## The Cylinder
The `Cylinder` structure describes a rectangle in two dimensions (use `Cuboid`
instead), or a cylinder in three dimensions. The principal axis is the positive
`y` axis.


| Description | Accessors | Value |
| --          | --       | --    |
| The half height of the cylinder | `c.half_height()` | User-defined by `Cylinder::new` |
| The radius of the cylinder basis | `c.radius()` | User-defined by `Cylinder::new` |
| The margin of the cylinder | `c.margin()` | `0.04` or user-defined by `Cylinder::new_with_margin` |

###### 3D example:
```rust
let cylinder1 = Cylinder::new(1.0, 0.5);
let cylinder2 = Cylinder::new_with_margin(1.0, 0.5, 0.1);

assert!(cylinder1.margin() == 0.04); // default margin
assert!(cylinder2.margin() == 0.1);  // user-defined margin
```

![fixme](cylinder3d_1)
![fixme](cylinder3d_2)


## The Cone
The `Cone` structure describes an isosceles triangle in two dimensions, or a
cone of revolution in tree dimensions. A cone is defined by the _radius_ of its
basis and its _half height_ − the half distance between the basis and the apex.
The principal axis is the positive `y` axis.

| Description | Accessors | Value |
| --          | --       | --    |
| The half height of the cone | `c.half_height()` | User-defined by `Cone::new` |
| The radius of the cone basis | `c.radius()`     | User-defined by `Cone::new` |
| The margin of the cone | `c.margin()`           | `0.04` or user-defined by `Cone::new_with_margin` |

###### 3D example:
```rust
let cone1 = Cone::new(1.0, 0.5);
let cone2 = Cone::new_with_margin(1.0, 0.5, 0.1);

assert!(cone1.margin() == 0.04); // default margin
assert!(cone2.margin() == 0.1);  // user-defined margin
```

![fixme](cone2d) ![fixme](cone3d)

## The Capsule
The `Capsule` structure describes the minkowski sum of a segment and a ball. In
other words, this is a cylinder with its flat extremities replaced by balls. A
capsule is defined by its _half height_ and the _radius_ of its extremities.
The principal axis is the positive `y` axis.

| Description | Accessors | Value |
| --          | --        | --    |
| The half height of the capsule | `c.half_height()` | User-defined by `Capsule::new` |
| The radius of the capsule extremities | `c.radius()` | User-defined by `Capsule::new` |

###### 2D and 3D example:
```rust
let capsule = Capsule::new(1.0, 0.5);

assert!(capsule.half_height() == 1.0);
assert!(capsule.radius() == 0.5);
```

![fixme](capsule2d) ![fixme](capsule3d)

## The Plane
The `Plane` structure describes a solid closed half-space. A plane is defined
by its _normal_. Every point that has a negative or zero dot product with the
plane normal is considered _inside_ of the plane. Other points are considered
_outside_ of the plane.

| Description | Accessors | Value |
| --          | --        | --    |
| The normal of the plane | `p.normal()` | User-defined by `Plane::new` |

###### 2D and 3D example:
```rust
let plane = Plane::new(Vec2::new(1.0, 0.0));

assert!(plane.normal().x == 1.0);
assert!(plane.normal().y == 0.0);
```

![fixme](plane2d) ![fixme](plane3d)


## The Mesh

<b> test </b>

###### 3D example:

###### 2D example:
