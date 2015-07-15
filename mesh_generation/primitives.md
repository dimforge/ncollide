# Primitives

Besides the `ToTriMesh` and `ToLinestrip` traits, the `procedural` and
`transformation` modules export free functions that generate various meshes and
line strips, including those accessible by the two former traits.

It also exposes functions to compute the convex hull of a set of point using
the [QuickHull algorithm](http://en.wikipedia.org/wiki/QuickHull) which has an
average $$O(n \log{n})$$ time complexity. There is currently not a single
convex hull algorithm implemented for any dimension, though individual
functions exist for the 2D and 3D cases:

| Function             | Description                                     |
| --                   | --                                              |
| `convex_hull2d(...)` | Computes the convex hull of a set of 2D points. |
| `convex_hull3d(...)` | Computes the convex hull of a set of 3D points. |

If you are not interested in the `Polyline` representation of the 2D convex
hull but only on the original indices of the vertices it contains, use the
`_idx` variant of the function − namely `convex_hull2d_idx(...)`.

## Example
The following example creates 100,000 random points and compute their
convex hull.

###### 2D example <span class="d2" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/primitives2d.rs')"></span>
```rust
let mut points = Vec::new();
for _ in range(0u, 100000) {
    points.push(rand::random::<Pnt2<f32>>() * 2.0f32);
}

let convex_hull = transformation::convex_hull2d(&points[..]);
```

###### 3D example <span class="d3" onclick="window.open('https://raw.githubusercontent.com/sebcrozet/ncollide/master/examples/primitives3d.rs')"></span>
```rust
let mut points = Vec::new();
for _ in range(0u, 100000) {
    points.push(rand::random::<Pnt3<f32>>() * 2.0f32);
}

let convex_hull = transformation::convex_hull3d(&points[..]);
```

<center>
![](../img/convex_hull3d.png)
</center>
