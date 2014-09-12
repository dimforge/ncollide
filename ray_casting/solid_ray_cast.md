# Solid ray cast

If the starting point of a ray is inside of a shape, the result depends on the
value of the `solid` flag. A solid ray cast (`solid` is set to `true`) will
return an intersection with its `toi` field set to zero and its `normal`
undefined. A non-solid (`solid` is set to `false`) ray cast will assume the
shape is hollow and will propagate on its inside until until hits a border:

<center>
![solid ray cast](../img/solid_ray_cast.svg)
</center>

If the starting point of the ray is outside of any geometry, then the `solid`
flag has no effect. Also note that a solid ray cast is usually **faster** than
the non-solid one.

## Example
The following example creates a cuboid and casts two rays on it. The first ray
starts inside of the cuboid and will return different time of impacts depending
on the solidity of the cast. The second ray should miss its target.

###### 2D example <div class="d2" onclick="window.open('../src/solid_ray_cast2d.rs')" /></div>

```rust
let cuboid     = Cuboid::new(Vec2::new(1.0, 2.0));
let ray_inside = Ray::new(na::zero(), Vec2::y());
let ray_miss   = Ray::new(Vec2::new(2.0, 2.0), Vec2::new(1.0, 1.0));

assert!(cuboid.toi_with_ray(&ray_inside, true).unwrap()  == 0.0);
assert!(cuboid.toi_with_ray(&ray_inside, false).unwrap() == 2.0);

assert!(cuboid.toi_with_ray(&ray_miss, false).is_none());
assert!(cuboid.toi_with_ray(&ray_miss, true).is_none());
```

###### 3D example <div class="d3" onclick="window.open('../src/solid_ray_cast3d.rs')" /></div>

```rust
let cuboid     = Cuboid::new(Vec3::new(1.0, 2.0, 1.0));
let ray_inside = Ray::new(na::zero(), Vec3::y());
let ray_miss   = Ray::new(Vec3::new(2.0, 2.0, 2.0), Vec3::new(1.0, 1.0, 1.0));

assert!(cuboid.toi_with_ray(&ray_inside, true).unwrap()  == 0.0);
assert!(cuboid.toi_with_ray(&ray_inside, false).unwrap() == 2.0);

assert!(cuboid.toi_with_ray(&ray_miss, false).is_none());
assert!(cuboid.toi_with_ray(&ray_miss, true).is_none());
```
