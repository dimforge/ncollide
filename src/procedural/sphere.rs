use super::utils;
#[cfg(feature = "dim2")]
use super::Polyline;
#[cfg(feature = "dim3")]
use super::{IndexBuffer, TriMesh};
use alga::general::Real;
use na;
#[cfg(feature = "dim3")]
use na::{Point2, Point3, Vector3};

/// Generates a UV sphere.
#[cfg(feature = "dim3")]
pub fn sphere<N>(
    diameter: N,
    ntheta_subdiv: u32,
    nphi_subdiv: u32,
    generate_uvs: bool,
) -> TriMesh<N>
where
    N: Real,
{
    let mut sphere = unit_sphere(ntheta_subdiv, nphi_subdiv, generate_uvs);

    sphere.scale_by_scalar(diameter);

    sphere
}

/// Generates a UV sphere centered at the origin and with a unit diameter.
#[cfg(feature = "dim3")]
pub fn unit_sphere<N>(ntheta_subdiv: u32, nphi_subdiv: u32, generate_uvs: bool) -> TriMesh<N>
where N: Real {
    if generate_uvs {
        unit_sphere_with_uvs(ntheta_subdiv, nphi_subdiv)
    } else {
        unit_sphere_without_uvs(ntheta_subdiv, nphi_subdiv)
    }
}

// FIXME: n{theta,phi}_subdiv are not the right names.
#[cfg(feature = "dim3")]
fn unit_sphere_without_uvs<N>(ntheta_subdiv: u32, nphi_subdiv: u32) -> TriMesh<N>
where N: Real {
    let pi = N::pi();
    let two_pi = N::two_pi();
    let pi_two = N::frac_pi_2();
    let dtheta = two_pi / na::convert(ntheta_subdiv as f64);
    let dphi = pi / na::convert(nphi_subdiv as f64);

    let mut coords = Vec::new();
    let mut curr_phi = -pi_two + dphi;

    // coords.
    coords.push(Point3::new(na::zero(), -na::one::<N>(), na::zero()));

    for _ in 0..nphi_subdiv - 1 {
        utils::push_circle(
            curr_phi.cos(),
            ntheta_subdiv,
            dtheta,
            curr_phi.sin(),
            &mut coords,
        );
        curr_phi = curr_phi + dphi;
    }

    coords.push(Point3::new(na::zero(), na::one(), na::zero()));

    // the normals are the same as the coords.
    let normals: Vec<Vector3<N>> = coords.iter().map(|p| p.coords).collect();

    // index buffer
    let mut idx = Vec::new();

    utils::push_degenerate_top_ring_indices(1, 0, ntheta_subdiv, &mut idx);

    utils::reverse_clockwising(&mut idx[..]);

    for i in 0..nphi_subdiv - 2 {
        let bottom = 1 + i * ntheta_subdiv;
        let up = bottom + ntheta_subdiv;
        utils::push_ring_indices(bottom, up, ntheta_subdiv, &mut idx);
    }

    utils::push_degenerate_top_ring_indices(
        1 + (nphi_subdiv - 2) * ntheta_subdiv,
        coords.len() as u32 - 1,
        ntheta_subdiv,
        &mut idx,
    );

    let mut res = TriMesh::new(coords, Some(normals), None, Some(IndexBuffer::Unified(idx)));

    let _0_5: N = na::convert(0.5);

    res.scale_by_scalar(_0_5);

    res
}

#[cfg(feature = "dim3")]
fn unit_sphere_with_uvs<N: Real>(ntheta_subdiv: u32, nphi_subdiv: u32) -> TriMesh<N> {
    let pi = N::pi();
    let two_pi = N::two_pi();
    let pi_two = N::frac_pi_2();
    let duvtheta = N::one() / na::convert(ntheta_subdiv as f64); // step of uv.x coordinates.
    let duvphi = N::one() / na::convert(nphi_subdiv as f64); // step of uv.y coordinates.
    let dtheta = two_pi * duvtheta;
    let dphi = pi * duvphi;

    let mut coords = Vec::new();
    let mut curr_phi = -pi_two;

    for _ in 0..nphi_subdiv + 1 {
        utils::push_circle(
            curr_phi.cos(),
            ntheta_subdiv + 1,
            dtheta,
            curr_phi.sin(),
            &mut coords,
        );
        curr_phi = curr_phi + dphi;
    }

    // the normals are the same as the coords
    let normals: Vec<Vector3<N>> = coords.iter().map(|p| p.coords).collect();

    // index buffer
    let mut idx = Vec::new();

    for i in 0..nphi_subdiv {
        let bottom = i * (ntheta_subdiv + 1);
        let up = bottom + (ntheta_subdiv + 1);
        utils::push_open_ring_indices(bottom, up, ntheta_subdiv + 1, &mut idx);
    }

    let mut uvs = Vec::new();
    let mut curr_uvphi = na::zero::<N>();

    for _ in 0..nphi_subdiv + 1 {
        let mut curr_uvtheta = na::zero::<N>();

        for _ in 0..ntheta_subdiv + 1 {
            uvs.push(Point2::new(curr_uvtheta, curr_uvphi));
            curr_uvtheta = curr_uvtheta + duvtheta;
        }

        curr_uvphi = curr_uvphi + duvphi;
    }

    let mut res = TriMesh::new(
        coords,
        Some(normals),
        Some(uvs),
        Some(IndexBuffer::Unified(idx)),
    );

    let _0_5: N = na::convert(0.5);
    res.scale_by_scalar(_0_5);

    res
}

/// Creates an hemisphere with a diameter of 1.
#[cfg(feature = "dim3")]
pub fn unit_hemisphere<N: Real>(ntheta_subdiv: u32, nphi_subdiv: u32) -> TriMesh<N> {
    let two_pi = N::two_pi();
    let pi_two = N::frac_pi_2();
    let dtheta = two_pi / na::convert(ntheta_subdiv as f64);
    let dphi = pi_two / na::convert(nphi_subdiv as f64);

    let mut coords = Vec::new();
    let mut curr_phi = na::zero::<N>();

    for _ in 0..nphi_subdiv - 1 {
        utils::push_circle(
            curr_phi.cos(),
            ntheta_subdiv,
            dtheta,
            curr_phi.sin(),
            &mut coords,
        );
        curr_phi = curr_phi + dphi;
    }

    coords.push(Point3::new(na::zero(), na::one(), na::zero()));

    let mut idx = Vec::new();

    for i in 0..nphi_subdiv - 2 {
        utils::push_ring_indices(
            i * ntheta_subdiv,
            (i + 1) * ntheta_subdiv,
            ntheta_subdiv,
            &mut idx,
        );
    }

    utils::push_degenerate_top_ring_indices(
        (nphi_subdiv - 2) * ntheta_subdiv,
        coords.len() as u32 - 1,
        ntheta_subdiv,
        &mut idx,
    );

    // Result
    let normals: Vec<Vector3<N>> = coords.iter().map(|p| p.coords).collect();
    // FIXME: uvs
    let mut out = TriMesh::new(coords, Some(normals), None, Some(IndexBuffer::Unified(idx)));

    // set the radius to 0.5
    let _0_5: N = na::convert(0.5);
    out.scale_by_scalar(_0_5);

    out
}

/// Creates a circle lying on the `(x,y)` plane.
#[cfg(feature = "dim2")]
pub fn circle<N: Real>(diameter: &N, nsubdivs: u32) -> Polyline<N> {
    let two_pi = N::two_pi();
    let dtheta = two_pi / na::convert(nsubdivs as f64);

    let mut pts = Vec::with_capacity(nsubdivs as usize);

    utils::push_xy_arc(*diameter / na::convert(2.0), nsubdivs, dtheta, &mut pts);

    // FIXME: normals

    Polyline::new(pts, None)
}

/// Creates a circle lying on the `(x,y)` plane.
#[cfg(feature = "dim2")]
pub fn unit_circle<N: Real>(nsubdivs: u32) -> Polyline<N> {
    // FIXME: do this the other way round?
    circle(&na::convert(1.0), nsubdivs)
}
