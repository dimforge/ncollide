# Composite shapes

**ncollide** supports shapes that are defined from other shapes:
1. The `Compound` shape describes a concave shape from convex ones.
2. The `Reflection` shape describes the reflection of another shape.
3. The `MinkowskiSum` and `AnnotatedMinkowskiSum` shapes describe the
   Minkowski sum of two shapes.

## Compound
The `Compound` structure is the main way of describing concave shapes from
convex ones. It differs from `Mesh` in that it is not a set of triangles but a
set of any shape implementing the `Shape` trait. Two steps are necessary to
create a `Compound` shape:
1. Initialize a `CompoundData` structure. Several shapes implementing the
   `Shape` trait together with a delta transformation matrix can be added to
   the compound data.
2. Call `Compound::new` with the initialized compound data.

| Method | Description |
| --          | --        |
| `.shapes()` | The shapes composing the compound. |
| `.bounding_volumes()` | The AABB of the shapes composing the compound. |
| `.bvt()` | The space-partitioning acceleration structure used by the compound. |

###### 2D example <span class="d2" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/compound2d.rs')"></span>
```rust
// Delta transformation matrices.
let delta1 = Iso2::new(Vec2::new(0.0f32, -1.5), na::zero());
let delta2 = Iso2::new(Vec2::new(-1.5f32, 0.0), na::zero());
let delta3 = Iso2::new(Vec2::new(1.5f32,  0.0), na::zero());

// 1) Initialize the shape list.
let mut shapes = Vec::new();
let horizontal_box = Arc::new(Box::new(Cuboid::new(Vec2::new(1.5f32,  0.25))) as Box<Repr2<f32>>);
let vertical_box   = Arc::new(Box::new(Cuboid::new(Vec2::new(0.25f32, 1.5))) as Box<Repr2<f32>>);

shapes.push((delta1, horizontal_box));
shapes.push((delta2, vertical_box.clone()));
shapes.push((delta3, vertical_box));

// 2) Create the compound shape.
let compound = Compound::new(shapes);

assert!(compound.shapes().len() == 3)
```

<center>
![fixme](../img/compound2d.png)
</center>

###### 3D example <span class="d3" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/compound3d.rs')"></span>
```rust
// Delta transformation matrices.
let delta1 = Iso3::new(Vec3::new(0.0f32, -1.5, 0.0), na::zero());
let delta2 = Iso3::new(Vec3::new(-1.5f32, 0.0, 0.0), na::zero());
let delta3 = Iso3::new(Vec3::new(1.5f32, 0.0,  0.0), na::zero());

// 1) Initialize the shape list.
let mut shapes = Vec::new();
let horizontal_box = Arc::new(Box::new(Cuboid::new(Vec3::new(1.5f32,  0.25, 0.25))) as Box<Repr3<f32>>);
let vertical_box   = Arc::new(Box::new(Cuboid::new(Vec3::new(0.25f32, 1.5, 0.25))) as Box<Repr3<f32>>);

shapes.push((delta1, horizontal_box));
shapes.push((delta2, vertical_box.clone()));
shapes.push((delta3, vertical_box));

// 2) Create the compound shape.
let compound = Compound::new(shapes);

assert!(compound.shapes().len() == 3)
```

<center>
![3d compound](../img/compound3d.png)
</center>

## Reflection
The `Reflection` structure describes the reflection of a shape $$\mathcal{A}$$
with regard to the origin:

$$
-\mathcal{A} = \left\{ -\mathbf{a} \mid{} \mathbf{a} \in \mathcal{A} \right\}
$$

Note that the reflected shape and the reflection itself are lifetime-bound.

| Method | Description |
| --       | --        |
| `.shape()` | The shape affected by the reflection. |

###### 2D and 3D example <span class="d3" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/reflection3d.rs')"></span><span class="sp"></span><span class="d2" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/reflection2d.rs')"></span>

```rust
let cone = Cone::new(0.5f32, 0.75);

// Build the reflection.
let _ = Reflection::new(&cone);
```

<center>
![reflected 2d cone](../img/refl2d.png) 
![reflected 3d cone](../img/refl3d.png)
</center>

## Minkowski Sum
The `MinkowskiSum` structure describes the Minkoswki sum of two shapes
implementing the `Implicit` trait. If $$\mathcal{A}$$ and $$\mathcal{B}$$ are
two (possibly infinite) sets of points their Minkowski sum is given by the set:

$$
\mathcal{A} \oplus \mathcal{B} = \left\{ \mathbf{a} + \mathbf{b} \mid{} \mathbf{a} \in \mathcal{A}, \mathbf{b} \in \mathcal{B} \right\}
$$

In other words, this is the union of all points of one shape successively
translated by each point of the other one. This is extremely useful for
discrete and continuous collision detection. Note that the shapes forming
the Minkowski sum are lifetime-bound with the Minkowski sum herself.

| Method | Description |
| --       | --        |
| `.m1()` | The local transformation of the **first** shape involved in the sum. |
| `.m2()` | The local transformation of the **second** shape involved in the sum. |
| `.g1()` | The **first** shape involved in the sum. |
| `.g2()` | The **second** shape involved in the sum. |

###### 2D and 3D example <span class="d3" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/minkowski_sum3d.rs')"></span><span class="sp"></span><span class="d2" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/minkowski_sum2d.rs')"></span>

```rust
let cylinder = Cylinder::new(0.5f32, 0.75);
let cone     = Cone::new(0.75f32, 0.75);

let delta_cylinder = na::one::<Iso3<f32>>(); // identity matrix.
let delta_cone     = na::one::<Iso3<f32>>(); // identity matrix.

let _ = MinkowskiSum::new(&delta_cylinder, &cylinder, &delta_cone, &cone);
```

<center>
![cylinder + cone](../img/msum2d.png)
![cylinder + cone](../img/msum3d.png)
</center>

###### Configuration Space Obstacle construction example <span class="d3" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/configuration_space_obstacle3d.rs')"></span><span class="sp"></span><span class="d2" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/configuration_space_obstacle2d.rs')"></span>
The Configuration Space Obstacle is the same as the Minkowski sum of the first
shape with the reflection of the second one:

$$
\mathcal{A} \ominus \mathcal{B} = \mathcal{A} \oplus -\mathcal{B} = \left\{ \mathbf{a} - \mathbf{b} \mid{} \mathbf{a} \in \mathcal{A}, \mathbf{b} \in \mathcal{B} \right\}
$$

It is very used in robotics and interference detection to compute all the
impossible positions of the center of mass of an object that evolves in a
complex environment. Note that this is obviously **not** reflexive. It can be
constructed using the `MinkowskiSum` and `Reflection` shapes:

```rust
let cylinder   = Cylinder::new(0.5f32, 0.75);
let cone       = Cone::new(0.75f32, 0.75);
let reflection = Reflection::new(&cone); // Take the reflection of the cone.

let delta_cylinder = na::one::<Iso3<f32>>(); // identity matrix.
let delta_cone     = na::one::<Iso3<f32>>(); // identity matrix.

// Build the Configuration Space Obstacle.
let _ = MinkowskiSum::new(&delta_cylinder, &cylinder, &delta_cone, &reflection);
```

<center>
![cylinder - cone](../img/cso2d.png) 
![cylinder - cone](../img/cso3d.png) 
</center>
