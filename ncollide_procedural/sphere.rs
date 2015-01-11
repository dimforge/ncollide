use na;
use na::{Pnt3, Vec3, Pnt2, BaseFloat};
use super::{Polyline, TriMesh, IndexBuffer};
use super::utils;
use math::{Scalar, Point, Vect};

/// Generates a UV sphere.
pub fn sphere<N>(diameter:      N,
                 ntheta_subdiv: u32,
                 nphi_subdiv:   u32,
                 generate_uvs:  bool)
                 -> TriMesh<N, Pnt3<N>, Vec3<N>>
    where N: Scalar {
    let mut sphere = unit_sphere(ntheta_subdiv, nphi_subdiv, generate_uvs);

    sphere.scale_by_scalar(diameter);

    sphere
}

/// Generates a UV sphere centered at the origin and with a unit diameter.
pub fn unit_sphere<N>(ntheta_subdiv: u32,
                      nphi_subdiv:   u32,
                      generate_uvs:  bool)
                      -> TriMesh<N, Pnt3<N>, Vec3<N>>
    where N: Scalar {
    if generate_uvs {
        unit_sphere_with_uvs(ntheta_subdiv, nphi_subdiv)
    }
    else {
        unit_sphere_without_uvs(ntheta_subdiv, nphi_subdiv)
    }
}

// FIXME: n{theta,phi}_subdiv are not the right names.
fn unit_sphere_without_uvs<N>(ntheta_subdiv: u32, nphi_subdiv: u32) -> TriMesh<N, Pnt3<N>, Vec3<N>>
    where N: Scalar {
    let pi: N     = BaseFloat::pi();
    let two_pi: N = BaseFloat::two_pi();
    let pi_two: N = BaseFloat::frac_pi_2();
    let dtheta    = two_pi / na::cast(ntheta_subdiv as f64);
    let dphi      = pi / na::cast(nphi_subdiv as f64);

    let mut coords   = Vec::new();
    let mut curr_phi = -pi_two + dphi;

    // coords.
    coords.push(Pnt3::new(na::zero(), -na::one::<N>(), na::zero()));

    for _ in range(0, nphi_subdiv - 1) {
        utils::push_circle(curr_phi.cos(), ntheta_subdiv, dtheta, curr_phi.sin(), &mut coords);
        curr_phi = curr_phi + dphi;
    }

    coords.push(Pnt3::new(na::zero(), na::one(), na::zero()));

    // the normals are the same as the coords.
    let normals: Vec<Vec3<N>> = coords.iter().map(|p| p.as_vec().clone()).collect();

    // index buffer
    let mut idx = Vec::new();

    utils::push_degenerate_top_ring_indices(1, 0, ntheta_subdiv, &mut idx);

    utils::reverse_clockwising(idx.as_mut_slice());

    for i in range(0, nphi_subdiv - 2) {
        let bottom = 1 + i * ntheta_subdiv;
        let up     = bottom + ntheta_subdiv;
        utils::push_ring_indices(bottom, up, ntheta_subdiv, &mut idx);
    }

    utils::push_degenerate_top_ring_indices(1 + (nphi_subdiv - 2) * ntheta_subdiv,
                                            coords.len() as u32 - 1,
                                            ntheta_subdiv,
                                            &mut idx);

    let mut res = TriMesh::new(coords, Some(normals), None, Some(IndexBuffer::Unified(idx)));

    let _0_5: N = na::cast(0.5);

    res.scale_by_scalar(_0_5);

    res
}

fn unit_sphere_with_uvs<N>(ntheta_subdiv: u32, nphi_subdiv: u32) -> TriMesh<N, Pnt3<N>, Vec3<N>>
    where N: Scalar {
    let pi: N     = BaseFloat::pi();
    let two_pi: N = BaseFloat::two_pi();
    let pi_two: N = BaseFloat::frac_pi_2();
    let duvtheta  = na::one::<N>() / na::cast(ntheta_subdiv as f64); // step of uv.x coordinates.
    let duvphi    = na::one::<N>() / na::cast(nphi_subdiv as f64);   // step of uv.y coordinates.
    let dtheta    =  two_pi * duvtheta;
    let dphi      =  pi * duvphi;

    let mut coords   = Vec::new();
    let mut curr_phi = -pi_two;

    for _ in range(0, nphi_subdiv + 1) {
        utils::push_circle(curr_phi.cos(), ntheta_subdiv + 1, dtheta, curr_phi.sin(), &mut coords);
        curr_phi = curr_phi + dphi;
    }

    // the normals are the same as the coords
    let normals: Vec<Vec3<N>> = coords.iter().map(|p| p.as_vec().clone()).collect();

    // index buffer
    let mut idx = Vec::new();

    for i in range(0, nphi_subdiv) {
        let bottom = i * (ntheta_subdiv + 1);
        let up     = bottom + (ntheta_subdiv + 1);
        utils::push_open_ring_indices(bottom, up, ntheta_subdiv + 1, &mut idx);
    }


    let mut uvs        = Vec::new();
    let mut curr_uvphi = na::zero::<N>();

    for _ in range(0, nphi_subdiv + 1) {
        let mut curr_uvtheta = na::zero::<N>();

        for _ in range(0, ntheta_subdiv + 1) {
            uvs.push(Pnt2::new(curr_uvtheta, curr_uvphi));
            curr_uvtheta = curr_uvtheta + duvtheta;
        }

        curr_uvphi = curr_uvphi + duvphi;
    }

    let mut res = TriMesh::new(coords, Some(normals), Some(uvs), Some(IndexBuffer::Unified(idx)));

    let _0_5: N = na::cast(0.5);
    res.scale_by_scalar(_0_5);

    res
}

/// Creates an hemisphere with a diameter of 1.
pub fn unit_hemisphere<N>(ntheta_subdiv: u32, nphi_subdiv: u32) -> TriMesh<N, Pnt3<N>, Vec3<N>>
    where N: Scalar {
    let two_pi: N = BaseFloat::two_pi();
    let pi_two: N = BaseFloat::frac_pi_2();
    let dtheta    =  two_pi / na::cast(ntheta_subdiv as f64);
    let dphi      =  pi_two / na::cast(nphi_subdiv as f64);

    let mut coords     = Vec::new();
    let mut curr_phi   = na::zero::<N>();

    for _ in range(0, nphi_subdiv - 1) {
        utils::push_circle(curr_phi.cos(), ntheta_subdiv, dtheta, curr_phi.sin(), &mut coords);
        curr_phi = curr_phi + dphi;
    }

    coords.push(Pnt3::new(na::zero(), na::one(), na::zero()));

    let mut idx = Vec::new();

    for i in range(0, nphi_subdiv - 2) {
        utils::push_ring_indices(i * ntheta_subdiv, (i + 1) * ntheta_subdiv, ntheta_subdiv, &mut idx);
    }

    utils::push_degenerate_top_ring_indices((nphi_subdiv - 2) * ntheta_subdiv,
                                            coords.len() as u32 - 1,
                                            ntheta_subdiv,
                                            &mut idx);

    // Result
    let normals: Vec<Vec3<N>> = coords.iter().map(|p| p.as_vec().clone()).collect();
    // FIXME: uvs
    let mut out = TriMesh::new(coords, Some(normals), None, Some(IndexBuffer::Unified(idx)));

    // set the radius to 0.5
    let _0_5: N = na::cast(0.5);
    out.scale_by_scalar(_0_5);

    out
}

/// Creates a circle lying on the `(x,y)` plane.
pub fn circle<N, P, V>(diameter: &N, nsubdivs: u32) -> Polyline<N, P, V>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    let two_pi: N = BaseFloat::two_pi();
    let dtheta    = two_pi / na::cast(nsubdivs as f64);

    let mut pts = Vec::with_capacity(nsubdivs as usize);

    utils::push_xy_arc(*diameter / na::cast(2.0), nsubdivs, dtheta, &mut pts);

    // FIXME: normals

    Polyline::new(pts, None)
}

/// Creates a circle lying on the `(x,y)` plane.
pub fn unit_circle<N, P, V>(nsubdivs: u32) -> Polyline<N, P, V>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    // FIXME: do this the other way round?
    circle::<N, P, V>(&na::cast(1.0), nsubdivs)
}
