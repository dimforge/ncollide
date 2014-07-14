use nalgebra::na;
use math::{Scalar, Vect};
use parametric::ParametricSurface;
use procedural::TriMesh;
use procedural;

#[dim3]
/// Meshing algorithm that uniformly triangulates the parametric space.
pub fn parametric_surface_uniform<S: ParametricSurface>(s:        &S,
                                                        usubdivs: uint,
                                                        vsubdivs: uint)
                                                        -> TriMesh<Scalar, Vect> {
    assert!(usubdivs > 0);
    assert!(vsubdivs > 0);

    let mut surface = procedural::unit_quad(usubdivs, vsubdivs);

    {
        let uvs     = surface.uvs.as_ref().unwrap().as_slice();
        let normals = surface.normals.as_mut().unwrap().as_mut_slice();
        let coords  = surface.coords.as_mut_slice();

        for j in range(0, vsubdivs + 1) {
            for i in range(0, usubdivs + 1) {
                let id            = i + j * (usubdivs + 1);
                let (s, s_u, s_v) = s.at_u_v(uvs[id].x, uvs[id].y);

                normals[id] = na::cross(&s_u, &s_v);
                coords[id]  = s;
            }
        }
    }

    surface
}

#[not_dim3]
/// Not yet implemented in dimensions other than 3.
pub fn parametric_surface_uniform<S: ParametricSurface>(_: &S, _: uint, _: uint) -> TriMesh<Scalar, Vect> {
    fail!("`parametric_surface_uniform` is not yet implemented for dimensions other than 3.")
}
