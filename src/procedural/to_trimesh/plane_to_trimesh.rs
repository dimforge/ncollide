use na;
use shape::Plane;
use procedural::{ToTriMesh, TriMesh};
use procedural;
use math::{N, Point, Vect};

#[cfg(feature = "3d")]
impl ToTriMesh<(u32, u32)> for Plane {
    fn to_trimesh(&self, (nwidth_subdiv, nheight_subdiv): (u32, u32)) -> TriMesh<N, Point,Vect> {
        let mut res = procedural::quad(na::one(), na::one(), nwidth_subdiv as uint, nheight_subdiv as uint);
        //                                           ^^^^^^^                 ^^^^^^^
        //                                    FIXME: This is not very consistant with
        //                                           the other functions.

        // `res` lies on the (0, x, y) plane. We have to align it with the plane normal.
        let mut axis = na::zero::<Vect>();
        axis.set(2, na::one());

        let daxis = na::cross(&axis, &self.normal());

        res.rotate_by(&daxis);

        res
    }
}

#[cfg(not(feature = "3d"))]
impl ToTriMesh<(u32, u32)> for Plane {
    fn to_trimesh(&self, (nwidth_subdiv, nheight_subdiv): (u32, u32)) -> TriMesh<N, Point,Vect> {
        procedural::quad(na::one(), na::one(), nwidth_subdiv as uint, nheight_subdiv as uint)
    }
}
