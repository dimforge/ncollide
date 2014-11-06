use na::{Pnt3, Vec3};
use na;
use shape::{Cuboid3, Cuboid3d};
use procedural::{ToTriMesh, TriMesh};
use procedural;

macro_rules! impl_to_trimesh_cuboid3(
    ($t: ty, $n: ty) => {
        impl ToTriMesh<$n, Pnt3<$n>, Vec3<$n>, ()> for $t {
            fn to_trimesh(&self, _: ()) -> TriMesh<$n, Pnt3<$n>, Vec3<$n>> {
                let _2: $n = na::cast(2.0f64);

                procedural::cuboid(&(*self.half_extents() * _2))
            }
        }
    }
)

impl_to_trimesh_cuboid3!(Cuboid3, f32)
impl_to_trimesh_cuboid3!(Cuboid3d, f64)

// FIXME: in 2d, generate a filled rectangle.
