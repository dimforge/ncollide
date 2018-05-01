//! Utilities useful for various generations tasks.

#[cfg(feature = "dim3")]
use std::iter;
#[cfg(feature = "dim3")]
use std::collections::HashMap;
#[cfg(feature = "dim3")]
use std::collections::hash_map::Entry;
#[cfg(feature = "dim3")]
use num::Zero;
#[cfg(feature = "dim3")]
use na;
use na::Real;
#[cfg(feature = "dim3")]
use utils::HashablePartialEq;
use math::{Point, Vector};

// FIXME: remove that in favor of `push_xy_circle` ?
/// Pushes a discretized counterclockwise circle to a buffer.
#[cfg(feature = "dim3")]
#[inline]
pub fn push_circle<N: Real>(radius: N, nsubdiv: u32, dtheta: N, y: N, out: &mut Vec<Point<N>>) {
    let mut curr_theta = N::zero();

    for _ in 0..nsubdiv {
        out.push(Point::new(
            curr_theta.cos() * radius,
            y.clone(),
            curr_theta.sin() * radius,
        ));
        curr_theta = curr_theta + dtheta;
    }
}

/// Pushes a discretized counterclockwise circle to a buffer.
/// The circle is contained on the plane spanned by the `x` and `y` axis.
#[inline]
pub fn push_xy_arc<N: Real>(radius: N, nsubdiv: u32, dtheta: N, out: &mut Vec<Point<N>>) {
    let mut curr_theta = N::zero();

    for _ in 0..nsubdiv {
        let mut pt_coords = Vector::zeros();

        pt_coords[0] = curr_theta.cos() * radius;
        pt_coords[1] = curr_theta.sin() * radius;
        out.push(Point::from_coordinates(pt_coords));

        curr_theta = curr_theta + dtheta;
    }
}

/// Creates the faces from two circles with the same discretization.
#[cfg(feature = "dim3")]
#[inline]
pub fn push_ring_indices(
    base_lower_circle: u32,
    base_upper_circle: u32,
    nsubdiv: u32,
    out: &mut Vec<Point<u32>>,
) {
    push_open_ring_indices(base_lower_circle, base_upper_circle, nsubdiv, out);

    // adjust the last two triangles
    push_rectangle_indices(
        base_upper_circle,
        base_upper_circle + nsubdiv - 1,
        base_lower_circle,
        base_lower_circle + nsubdiv - 1,
        out,
    );
}

/// Creates the faces from two circles with the same discretization.
#[cfg(feature = "dim3")]
#[inline]
pub fn push_open_ring_indices(
    base_lower_circle: u32,
    base_upper_circle: u32,
    nsubdiv: u32,
    out: &mut Vec<Point<u32>>,
) {
    assert!(nsubdiv > 0);

    for i in 0..nsubdiv - 1 {
        let bli = base_lower_circle + i;
        let bui = base_upper_circle + i;
        push_rectangle_indices(bui + 1, bui, bli + 1, bli, out);
    }
}

/// Creates the faces from a circle and a point that is shared by all triangle.
#[cfg(feature = "dim3")]
#[inline]
pub fn push_degenerate_top_ring_indices(
    base_circle: u32,
    point: u32,
    nsubdiv: u32,
    out: &mut Vec<Point<u32>>,
) {
    push_degenerate_open_top_ring_indices(base_circle, point, nsubdiv, out);

    out.push(Point::new(base_circle + nsubdiv - 1, point, base_circle));
}

/// Creates the faces from a circle and a point that is shared by all triangle.
#[cfg(feature = "dim3")]
#[inline]
pub fn push_degenerate_open_top_ring_indices(
    base_circle: u32,
    point: u32,
    nsubdiv: u32,
    out: &mut Vec<Point<u32>>,
) {
    assert!(nsubdiv > 0);

    for i in 0..nsubdiv - 1 {
        out.push(Point::new(base_circle + i, point, base_circle + i + 1));
    }
}

/// Pushes indices so that a circle is filled with triangles. Each triangle will have the
/// `base_circle` point in common.
/// Pushes `nsubdiv - 2` elements to `out`.
#[cfg(feature = "dim3")]
#[inline]
pub fn push_filled_circle_indices(base_circle: u32, nsubdiv: u32, out: &mut Vec<Point<u32>>) {
    for i in base_circle + 1..base_circle + nsubdiv - 1 {
        out.push(Point::new(base_circle, i, i + 1));
    }
}

/// Given four corner points, pushes to two counterclockwise triangles to `out`.
///
/// # Arguments:
/// * `ul` - the up-left point.
/// * `dl` - the down-left point.
/// * `dr` - the down-left point.
/// * `ur` - the up-left point.
#[cfg(feature = "dim3")]
#[inline]
pub fn push_rectangle_indices(ul: u32, ur: u32, dl: u32, dr: u32, out: &mut Vec<Point<u32>>) {
    out.push(Point::new(ul.clone(), dl, dr.clone()));
    out.push(Point::new(dr, ur, ul));
}

/// Reverses the clockwising of a set of faces.
#[cfg(feature = "dim3")]
#[inline]
pub fn reverse_clockwising(indices: &mut [Point<u32>]) {
    for i in indices.iter_mut() {
        i.coords.swap((0, 0), (1, 0));
    }
}

/// Duplicates the indices of each triangle on the given index buffer.
///
/// For example: [ (0.0, 1.0, 2.0) ] becomes: [ (0.0, 0.0, 0.0), (1.0, 1.0, 1.0), (2.0, 2.0, 2.0)].
#[cfg(feature = "dim3")]
#[inline]
pub fn split_index_buffer(indices: &[Point<u32>]) -> Vec<Point<Point<u32>>> {
    let mut resi = Vec::new();

    for vertex in indices.iter() {
        resi.push(Point::new(
            Point::new(vertex.x, vertex.x, vertex.x),
            Point::new(vertex.y, vertex.y, vertex.y),
            Point::new(vertex.z, vertex.z, vertex.z),
        ));
    }

    resi
}

/// Duplicates the indices of each triangle on the given index buffer, giving the same id to each
/// identical vertex.
#[cfg(feature = "dim3")]
#[inline]
pub fn split_index_buffer_and_recover_topology<N: Real>(
    indices: &[Point<u32>],
    coords: &[Point<N>],
) -> (Vec<Point<Point<u32>>>, Vec<Point<N>>) {
    let mut vtx_to_id = HashMap::new();
    let mut new_coords = Vec::with_capacity(coords.len());
    let mut out = Vec::with_capacity(indices.len());

    fn resolve_coord_id<N: Real>(
        coord: &Point<N>,
        vtx_to_id: &mut HashMap<HashablePartialEq<Point<N>>, u32>,
        new_coords: &mut Vec<Point<N>>,
    ) -> u32 {
        let key = unsafe { HashablePartialEq::new(coord.clone()) };
        let id = match vtx_to_id.entry(key) {
            Entry::Occupied(entry) => entry.into_mut(),
            Entry::Vacant(entry) => entry.insert(new_coords.len() as u32),
        };

        if *id == new_coords.len() as u32 {
            new_coords.push(coord.clone());
        }

        *id
    }

    for t in indices.iter() {
        let va = resolve_coord_id(&coords[t.x as usize], &mut vtx_to_id, &mut new_coords);
        let oa = t.x;

        let vb = resolve_coord_id(&coords[t.y as usize], &mut vtx_to_id, &mut new_coords);
        let ob = t.y;

        let vc = resolve_coord_id(&coords[t.z as usize], &mut vtx_to_id, &mut new_coords);
        let oc = t.z;

        out.push(Point::new(
            Point::new(va, oa, oa),
            Point::new(vb, ob, ob),
            Point::new(vc, oc, oc),
        ));
    }

    new_coords.shrink_to_fit();

    (out, new_coords)
}

// FIXME: check at compile-time that we are in 3D?
/// Computes the normals of a set of vertices.
#[cfg(feature = "dim3")]
#[inline]
pub fn compute_normals<N: Real>(
    coordinates: &[Point<N>],
    faces: &[Point<u32>],
    normals: &mut Vec<Vector<N>>,
) {
    let mut divisor: Vec<N> = iter::repeat(na::zero()).take(coordinates.len()).collect();

    // Shrink the output buffer if it is too big.
    if normals.len() > coordinates.len() {
        normals.truncate(coordinates.len())
    }

    // Reinit all normals to zero.
    normals.clear();
    normals.extend(iter::repeat(na::zero::<Vector<N>>()).take(coordinates.len()));

    // Accumulate normals ...
    for f in faces.iter() {
        let edge1 = coordinates[f.y as usize] - coordinates[f.x as usize];
        let edge2 = coordinates[f.z as usize] - coordinates[f.x as usize];
        let cross = edge1.cross(&edge2);
        let normal;

        if !cross.is_zero() {
            normal = na::normalize(&cross)
        } else {
            normal = cross
        }

        normals[f.x as usize] = normals[f.x as usize] + normal;
        normals[f.y as usize] = normals[f.y as usize] + normal;
        normals[f.z as usize] = normals[f.z as usize] + normal;

        divisor[f.x as usize] = divisor[f.x as usize] + na::one();
        divisor[f.y as usize] = divisor[f.y as usize] + na::one();
        divisor[f.z as usize] = divisor[f.z as usize] + na::one();
    }

    // ... and compute the mean
    for (n, divisor) in normals.iter_mut().zip(divisor.iter()) {
        *n = *n / *divisor
    }
}
