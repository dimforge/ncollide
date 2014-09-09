# Mesh generation

While not directly part of the collision detection problem, mesh generation is
useful to extend the range of geometries supported by **ncollide** by
discretizing them such that they can be approximated with a `geom::Mesh`, a
`geom::Convex`, and/or a `geom::Compound`. It is also useful to obtain a
renderer-compliant representation of non-polyhedral models such that balls,
capsules, parametric surfaces, etc.


No mater the chosen version of **ncollide**, the two main types of mesh
generation results are `procedural::TriMesh` and `procedural::Polyline`. They
are both generic with regard the vector and scalar type.

### The TriMesh
The `TriMesh` structure describes a triangle mesh with optional per-vertex
normals and texture coordinates:


| Field     | Description                          |
|--         | --                                   |
| `coords`  | The vertex coordinates.              |
| `normals` | The vertex normals.                  |
| `uvs`     | The vertex texture coordinates.      |
| `indices` | The triangles vertices index buffer. |

The index buffer may take two forms. In its `procedural::UnifiedIndexBuffer`
form, the index of a triangle vertex is the same as the index identifying its
normal and uvs. While not very memory-efficient, this is the required
representation for renderers based on e.g. OpenGL:



In its `procedural::SplitIndexBuffer` form, each triangle vertex have three
indices: one for its position, one for its normal, and one for its texture
coordinate. This representation is usually topologically more interesting than
the unified index buffer:



It is possible (but time-consuming) to switch between those two kinds of index
buffers:

| Method                  | Description                     |
|--                       | --                              |
| `.unify_index_buffer()` | Transforms the current `TriMesh` index buffer to a `UnifiedIndexBuffer`. This copies the relevant data in order to make sure that every attributes of a single vertex is accessible using the same index. |
| `.split_index_buffer(heal)` | Transforms the current `TriMesh` index buffer to a `SplitIndexBuffer`. If `heal` is set to `true` it will try to recover the original mesh topology, identifying with the same index vertices having the exact same position. |


### The Polyline

The `Polyline` is a much simpler data structure than the `TriMesh`. It does not
have an index or texture coordinates buffer:

| Field     | Description             |
|--         | --                      |
| `coords`  | The vertex coordinates. |
| `normals` | The vertex normals.     |

Since it does not have an index buffer `coords` is assumed to be a line strip,
and there is no way to let two different vertices share the same normal. This
structure is not very practical to model complex 2D shapes. That is why it
should be changed to a more traditional index-buffer based representation in
future version of **ncollide**.


## Traits

The `procedural` module exposes two traits that allows the conversion from
non-polyhedral geometries to triangle meshes or polylines. First, the
`procedural::ToTriMesh` allows the creation of a `TriMesh` for an instance of
the implementor:

| Method           | Description                                    |
|--                | --                                             |
| `.to_trimesh(i)` | Creates a polyhedral representation of `self`. |

Similarly, import the `procedural::ToPolyline` trait to generate a `Polyline`
from an instance of the implementor:

| Method            | Description                                      |
|--                 | --                                               |
| `.to_polyline(i)` | Creates a polylineical representation of `self`. |

Note that those methods require a parameter that allows you to control the
discretization process. The exact type of this parameter depends on the type
that implements the trait. For example, a `Ball` requires two integers (one for
the subdivision of each spherical coordinate) to be transformed to a `TriMesh`;
the `Cuboid` however requires a parameter equal to `()` because it does not
require any user-defined informations to be discretized.
