# Bounding volumes

Performing some tests on an approximation of the shape of an object is often
useful to fasten several geometric queries.


For example testing two convex polyhedrons for intersection is a very
time-consuming operation. Instead, we could test that their spherical
approximations (namely, their bounding spheres) intersect; and if the
approximations fails this intersection test there is no need to perform the
same query on the original polyhedra. This test-on-the-approximations-first
method is called _prunning_.


The approximations presented here are conservative with regard to the object
volume−that is−the approximated shape is completely contained inside of the
approximating object. This is called a bounding volume.  There are many
possible bounding volumes.  The following figure shows a 2D polygon bounded by
a bounding sphere, an Axis Aligned Bounding Box (AABB), an Oriented Bounding
Box (OBB), and a convex hull:

<center>
![bounding volumes](../img/bounding_volumes.svg)
</center>

Currently, **ncollide** only supports [Bounding
Spheres](../bounding_volumes/bounding_sphere.html) and
[AABB](../bounding_volumes/aabb.html). Also note that bounding volumes are very
different from regular shapes: their position in space is completely contained
by the bounding volume structure (no need to use them together with a
transformation matrix).


## Traits

Bounding volumes must implement the `bounding_volume::BoundingVolume` trait:


| Method            | Description |
|--                 | --          |
| `.intersects(bv)` | Checks `self` for intersection with `bv`.              |
| `.contains(bv)`   | Returns `true` if `bv` is completely inside of `self`. |
| `.merge(bv)`      | Merge `self` and `bv` in place. |
| `.merged(bv)`     | Returns a bounding volume, result of the merge of `self` with `bv`. |
| `.loosen(m)`      | Dilates `self` by a ball of radius `m` in place.          |
| `.loosened(m)`    | Returns a copy of `self` dilated by a ball of radius `m`. |
| `.tighten(m)`     | Erodes `self` by a ball of radius `m` in place.          |
| `.tightened(m)`   | Returns a copy of `self` eroded by a ball of radius `m`. |

The `.loosen(...)` and `.loosened(...)` (resp. `.tighten(...)` and
`.tightened(...)`) methods allow you to dilate (resp. erode) the bounding
volume by a given margin. This will effectively make the new bounding volume
strictly larger (resp. thinner) than the original one.  This is useful e.g.
to optimize some [broad phase](../contact_determination/broad_phase.html)
algorithms.


Finally, some algorithm work with objects that can compute their own bounding
volumes all by themselves. This requirement is exposed by the
`bounding_volume::HasBoundingVolume` trait which is parametrized by the type of
the returned bounding volume:

| Method               | Description |
| --                   | --          |
| `.bounding_volume()` | Computes the bounding volume of `self`. |
