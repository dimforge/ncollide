use na;
use na::{Pnt3, Pnt2};
use super::{TriMesh, IndexBuffer};
use math::{Scalar, Point, Vect};

/// Adds a double-sided quad to the scene.
///
/// The quad is initially centered at (0, 0, 0). Its normal is the `z` axis. The quad itself is
/// composed of a user-defined number of triangles regularly spaced on a grid. This is the main way
/// to draw height maps.
///
/// # Arguments
/// * `w` - the quad width.
/// * `h` - the quad height.
/// * `usubdivs` - number of horizontal subdivisions. This correspond to the number of squares
/// which will be placed horizontally on each line. Must not be `0`.
/// * `vsubdivs` - number of vertical subdivisions. This correspond to the number of squares
/// which will be placed vertically on each line. Must not be `0`.
pub fn quad<N, P, V>(width: N, height: N, usubdivs: uint, vsubdivs: uint) -> TriMesh<N, P, V>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    let mut quad = unit_quad::<N, P, V>(usubdivs, vsubdivs);

    let mut s = na::zero::<V>();
    s[0] = width;
    s[1] = height;

    for i in range(2, na::dim::<V>()) {
        s[i] = na::one();
    }

    quad.scale_by(&s);

    quad
}

/// Adds a double-sided quad with the specified grid of vertices.
///
/// Normals are automatically computed.
///
/// # Arguments
/// * `nhpoints` - number of columns on the grid.
/// * `nvpoints` - number of lines on the grid.
pub fn quad_with_vertices<N, P, V>(vertices: &[P], nhpoints: uint, nvpoints: uint) -> TriMesh<N, P, V>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    assert!(nhpoints > 1 && nvpoints > 1, "The number of points must be at least 2 in each dimension.");

    let mut res = unit_quad::<N, P, V>(nhpoints - 1, nvpoints - 1);

    for (dest, src) in res.coords.iter_mut().zip(vertices.iter()) {
        *dest = src.clone();
    }

    res
}

/// Adds a double-sided quad with unit size to the scene.
///
/// The quad is initially centered at (0, 0, 0). Its normal is the `z` axis. The quad itself is
/// composed of a user-defined number of triangles regularly spaced on a grid. This is the main way
/// to draw height maps.
///
/// # Arguments
/// * `usubdivs` - number of horizontal subdivisions. This correspond to the number of squares
/// which will be placed horizontally on each line. Must not be `0`.
/// * `vsubdivs` - number of vertical subdivisions. This correspond to the number of squares
/// which will be placed vertically on each line. Must not be `0`.
pub fn unit_quad<N, P, V>(usubdivs: uint, vsubdivs: uint) -> TriMesh<N, P, V>
    where N: Scalar,
          P: Point<N, V>,
          V: Vect<N> {
    assert!(usubdivs > 0 && vsubdivs > 0, "The number of subdivisions cannot be zero");
    assert!(na::dim::<V>() >= 2);

    let wstep    = na::one::<N>() / na::cast(usubdivs as f64);
    let hstep    = na::one::<N>() / na::cast(vsubdivs as f64);
    let cw       = na::cast(0.5);
    let ch       = na::cast(0.5);

    let mut vertices   = Vec::new();
    let mut normals    = Vec::new();
    let mut triangles  = Vec::new();
    let mut tex_coords = Vec::new();

    // create the vertices
    for i in range(0u, vsubdivs + 1) {
        for j in range(0u, usubdivs + 1) {
            let ni: N = na::cast(i as f64);
            let nj: N = na::cast(j as f64);

            let mut v = na::orig::<P>();
            v[0] = nj * wstep - cw;
            v[1] = ni * hstep - ch;
            vertices.push(v);
            tex_coords.push(Pnt2::new(na::one::<N>() - nj * wstep, na::one::<N>() - ni * hstep))
        }
    }

    // create the normals
    for _ in range(0, (vsubdivs + 1) * (usubdivs + 1)) {
        let mut n = na::zero::<V>();
        n[0] = na::one();
        normals.push(n)
    }

    // create triangles
    fn dl_triangle(i: u32, j: u32, ws: u32) -> Pnt3<u32> {
        Pnt3::new((i + 1) * ws + j, i * ws + j, (i + 1) * ws + j + 1)
    }

    fn ur_triangle(i: u32, j: u32, ws: u32) -> Pnt3<u32> {
        Pnt3::new(i * ws + j, i * ws + (j + 1), (i + 1) * ws + j + 1)
    }

    for i in range(0u, vsubdivs) {
        for j in range(0u, usubdivs) {
            // build two triangles...
            triangles.push(dl_triangle(i as u32, j as u32, (usubdivs + 1) as u32));
            triangles.push(ur_triangle(i as u32, j as u32, (usubdivs + 1) as u32));
        }
    }

    TriMesh::new(vertices, Some(normals), Some(tex_coords), Some(IndexBuffer::Unified(triangles)))
}
