# Primitives

Besides the `ToTriMesh` and `ToLinestrip` traits, the `procedural` module
exports free functions that generates various meshes and line strips, including
those accessible by the two former traits.

It also exposes functions to compute the convex hull of a set of point using
the [QuickHull algorithm](http://en.wikipedia.org/wiki/QuickHull) which has an
average $$O(n \log{n})$$ time complexity.  Because computing the convex hull of
2d and 3d geometries is so common, two functions are always available, no
matter the number of dimensions you chose to compile **ncollide** for:

| Function             | Description                                     |
| --                   | --                                              |
| `convex_hull2d(...)` | Computes the convex hull of a set of 2D points. |
| `convex_hull3d(...)` | Computes the convex hull of a set of 3D points. |

If you are not interested on the `TriMesh` representation of the convex hull,
but only on the indices of the vertices contained by the convex hull, use the
`_idx` variant of those functions, namely `convex_hull2d_idx(...)` and
`convex_hull3d_idx(...)`.

## Example
Le following example creates 100,000 random points and compute their
convex hull.

###### 2D example <div class="d2" onclick="window.open('../src/primitives2d.rs')"></div>
```rust
let mut points = Vec::new();
for _ in range(0u, 100000) {
    points.push(rand::random::<Vec2<f32>>() * 2.0f32);
}

let convex_hull = procedural::convex_hull2d(points.as_slice());
```

###### 3D example <div class="d3" onclick="window.open('../src/primitives3d.rs')"></div>
```rust
let mut points = Vec::new();
for _ in range(0u, 100000) {
    points.push(rand::random::<Vec3<f32>>() * 2.0f32);
}

let convex_hull = procedural::convex_hull3d(points.as_slice());
```

<center>
![](../img/convex_hull3d.png)
</center>
