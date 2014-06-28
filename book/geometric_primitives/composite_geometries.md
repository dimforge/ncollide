# Composite geometries

**ncollide** supports geometries that are defined from other geometries:
1. the `Compound` geometry that describes a concave geometry from convex ones.
2. the `Reflection` geometry that describes the reflection of another geometry.
3. the `MinkowskiSum` and `AnnotatedMinkowskiSum` geometries that describe the
   minkowski sum of two geometries.

## The Compound
The `Compound` structure is the main way of describing concave geometries from
convex ones. It differs from `Mesh` in that it is not a set of triangles but a
set of other geometries supported by **ncollide**.  Two steps are necessary to
create a `Compound` geometry:
1. Initialize a `CompoundData` structure. Several geometries implementing the
   `Geom` trait, together with a delta transformation matrix can be added to
   the compound data.
2. Call `Compound::new` with the initialized data.

| Description | Accessors | Value |
| --          | --        | --     |
| The geometries composing this compound | `c.geoms()` | User-defined by `Compound::new` |
| The AABBs of the ceometries composing this compound | `c.bounding_volumes()` | Automatically computed |
| The space-partitioning accerelation structure used by this compound | `c.bvt()` | Automatically computed |

###### 2d example:
```rust
// delta transformation matrices.
let delta1 = Iso2::new(Vec2::new(0.0f32, -5.0), na::zero());
let delta2 = Iso2::new(Vec2::new(-5.0f32, 0.0), na::zero());
let delta3 = Iso2::new(Vec2::new(5.0f32,  0.0), na::zero());

// 1) initialize the CompoundData.
let mut compound_data = CompoundData::new();
compound_data.push_geom(delta1, Cuboid::new(Vec2::new(5.0f32, 0.75)), 1.0);
compound_data.push_geom(delta2, Cuboid::new(Vec2::new(0.75f32, 5.0)), 1.0);
compound_data.push_geom(delta3, Cuboid::new(Vec2::new(0.75f32, 5.0)), 1.0);

// 2) create the compound geometry.
let compound = Compound::new(compound_data);
```

![fixme](example2d)

###### 3d example:
```rust
// delta transformation matrices.
let delta1 = Iso3::new(Vec3::new(0.0f32, -5.0, 0.0), na::zero());
let delta2 = Iso3::new(Vec3::new(-5.0f32, 0.0, 0.0), na::zero());
let delta3 = Iso3::new(Vec3::new(5.0f32, 0.0, 0.0), na::zero());

// 1) initialize the CompoundData.
let mut compound_data = CompoundData::new();
compound_data.push_geom(delta1, Cuboid::new(Vec3::new(5.0f32, 0.25, 0.25)), 1.0);
compound_data.push_geom(delta2, Cuboid::new(Vec3::new(0.25f32, 5.0, 0.25)), 1.0);
compound_data.push_geom(delta3, Cuboid::new(Vec3::new(0.25f32, 5.0, 0.25)), 1.0);

// 2) create the compound geometry.
let compound = Compound::new(compound_data);
```

![fixme](example3d)

#### More about CompoundData
The previous examples show the simplest way of initializing the `CompoundData`
structure. However using the `compound_data.push_geom(...)` method works only
for geometries that implement both the `Geom` **and** `Volumetric` traits. In
    addition every geometry added with `push_geom(...)` are moved out. To save
    memory, we might want those to be shared by multiple composite geometries.
    Therefore, there are three ways of adding a geometry to a `CompoundData`:

1. `push_geom(...)`: use this if `geometry`
   implements `Volumetric` and does *not* have to be shared.
2. `push_geom_with_mass_properties(...)`: use this if `geometry` does *not*
   implement `Volumetric` and does *not* have to be shared. This time, the
   object mass, center of mass and angular inertia tensor must be providen.
3. `push_shared_geom_with_mass_properties(...)`: use this if `shared_geometry`
   has to be shared.  This time, even if `shared_geometry` did implement the
   `Volumetric` trait, the object mass, center of mass and angular inertia
   tensor must be providen.

###### 2d example:
```rust
```rust
// delta transformation matrices.
let delta1 = Iso2::new(Vec2::new(0.0f32, -5.0), na::zero());
let delta2 = Iso2::new(Vec2::new(-5.0f32, 0.0), na::zero());
let delta3 = Iso2::new(Vec2::new(5.0f32,  0.0), na::zero());

// 1) initialize the CompoundData.
let mut compound_data = CompoundData::new();

/*
 * push_geom
 */
// The mass, center of mass and angular inertia tensor are automatically
// computed.
compound_data.push_geom(delta1, Cuboid::new(Vec2::new(5.0f32, 0.75)), 1.0);

/*
 * push_geom_with_mass_properties
 */
// mass = 10.0
// center of mass = the origin (na::zero())
// angular inertia tensor = identity matrix (na::one())
compound_data.push_geom(delta2, Plane::new(Vec2::new(1f32, 0.0)), (10.0, na::zero(), na::one()));

/*
 * push_shared_geom_with_mass_properties
 */
// The geometry we want to share.
let cuboid = Cuboid::new(Vec2::new(0.75f32, 5.0);
// Make ncollide compute the mass properties of the cuboid.
let mass_properties = cuboid.mass_properties();
// Build the shared geometry.
let shared_cuboid = Rc::new(box  as Box<Cuboid>));
// Add the geometry to the compound data.
compound_data.push_geom(delta3, shared_cuboid.clone(), mass_properties);
// `shared_cuboid` can still be used thereafter…

// 2) create the compound geometry.
let compound = Compound::new(compound_data);
```

![fixme](example2d)

## The Reflection
The `Reflection` structure describes the reflection of another geometry with
regard to the origin. Note that the reflected geometry and the reflection
itself are lifetime-bound.

| Description | Accessors | Value |
| --          | --        | --    |
| The geometry affected by the reflection | `r.geom()` | User-defined by `Reflection::new` |

###### 2d and 3d example:
```rust
let cone       = Cone::new_with_margin(1.0, 0.5, 0.0);
let reflection = Reflection::new(&cone);
```

![fixme](reflected_cone_2d) ![fixme](reflected_cone_3d)

## The MinkowskiSum
The `MinkowskiSum` structure describes the minkoswki sum of two geometries
implementing the `Implicit` trait. This is extremely useful for discrete and
continious collision detection.  Note that the geometries forming the minkowski
sum are lifetime-bound with the minkowski sum herself. `MinkoswkiSum`, used in
pair with `Reflection` makes it easy to build the so-called Configuration Space
Obstacle.

| Description | Accessors | Value |
| --          | --        | --    |
| The local transformation of the **first** geometry involved in the sum.  | `m.m1()` | User-defined by `MinkowskiSum::new` |
| The local transformation of the **second** geometry involved in the sum. | `m.m2()` | User-defined by `MinkowskiSum::new` |
| The **first** geometry involved in the sum.  | `m.g1()` | User-defined by `MinkowskiSum::new` |
| The **second** geometry involved in the sum.  | `m.g2()` | User-defined by `MinkowskiSum::new` |

###### 2d and 3d example:
```rust
let capsule = Capsule::new(1.0, 0.5);
let cone    = Cone::new_with_margin(1.0, 0.5, 0.0);

let delta_capsule = na::one(); // identity matrix.
let delta_cone    = na::one(); // identity matrix.

// Build the Minkowski sum.
let sum = MinkoskiSum::new(&delta_capsule, &capsule, &delta_cone, &cone);
```

![fixme](sum2d) ![fixme](sum3d)

###### Configuration Space Obstacle construction example:
The Configuration Space Obstacle is the same as the Minkowski sum of the first
geometry with the reflection of the second one.

```rust
let capsule    = Capsule::new(1.0, 0.5);
let cone       = Cone::new_with_margin(1.0, 0.5, 0.0);
let reflection = Reflection::new(cone); // take the reflection of the cone.

let delta_capsule = na::one(); // identity matrix.
let delta_cone    = na::one(); // identity matrix.

// Build the Configuration Space Obstacle.
let cso = MinkoskiSum::new(&delta_capsule, &capsule, &delta_cone, &reflection);
```
