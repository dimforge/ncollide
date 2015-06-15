//! Utilities useful for various generations tasks.

use std::ops::{Index, IndexMut};
use std::mem;
use std::iter;
use std::collections::HashMap;
use std::collections::hash_map::Entry;
use num::Float;
use na;
use na::{Pnt3, Dim, Cross, Orig};
use ncollide_utils::{HashablePartialEq, AsBytes};
use math::{Scalar, Point, Vect};

// FIXME: remove that in favor of `push_xy_circle` ?
/// Pushes a discretized counterclockwise circle to a buffer.
#[inline]
pub fn push_circle<N: Scalar>(radius: N, nsubdiv: u32, dtheta: N, y: N, out: &mut Vec<Pnt3<N>>) {
    let mut curr_theta: N = na::zero();

    for _ in 0 .. nsubdiv {
        out.push(Pnt3::new(curr_theta.cos() * radius, y.clone(), curr_theta.sin() * radius));
        curr_theta = curr_theta + dtheta;
    }
}

/// Pushes a discretized counterclockwise circle to a buffer.
/// The circle is contained on the plane spanned by the `x` and `y` axis.
#[inline]
pub fn push_xy_arc<N, P>(radius: N, nsubdiv: u32, dtheta: N, out: &mut Vec<P>)
    where N: Scalar,
          P: Dim + Orig + Index<usize, Output = N> + IndexMut<usize, Output = N> {
    assert!(na::dim::<P>() >= 2);

    let mut curr_theta: N = na::zero();

    for _ in 0 .. nsubdiv {
        let mut pt = na::orig::<P>();

        pt[0] = curr_theta.cos() * radius;
        pt[1] = curr_theta.sin() * radius;
        out.push(pt);

        curr_theta = curr_theta + dtheta;
    }
}

/// Creates the faces from two circles with the same discretization.
#[inline]
pub fn push_ring_indices(base_lower_circle: u32,
                         base_upper_circle: u32,
                         nsubdiv:           u32,
                         out:               &mut Vec<Pnt3<u32>>) {
    push_open_ring_indices(base_lower_circle, base_upper_circle, nsubdiv, out);

    // adjust the last two triangles
    push_rectangle_indices(base_upper_circle, base_upper_circle + nsubdiv - 1,
                           base_lower_circle, base_lower_circle + nsubdiv - 1, out);
}

/// Creates the faces from two circles with the same discretization.
#[inline]
pub fn push_open_ring_indices(base_lower_circle: u32,
                              base_upper_circle: u32,
                              nsubdiv:           u32,
                              out:               &mut Vec<Pnt3<u32>>) {
    assert!(nsubdiv > 0);

    for i in 0 .. nsubdiv - 1 {
        let bli = base_lower_circle + i;
        let bui = base_upper_circle + i;
        push_rectangle_indices(bui + 1, bui,
                               bli + 1, bli, out);
    }
}

/// Creates the faces from a circle and a point that is shared by all triangle.
#[inline]
pub fn push_degenerate_top_ring_indices(base_circle: u32,
                                        point:       u32,
                                        nsubdiv:     u32,
                                        out:         &mut Vec<Pnt3<u32>>) {
    push_degenerate_open_top_ring_indices(base_circle, point, nsubdiv, out);

    out.push(Pnt3::new(base_circle + nsubdiv - 1, point, base_circle));
}

/// Creates the faces from a circle and a point that is shared by all triangle.
#[inline]
pub fn push_degenerate_open_top_ring_indices(base_circle: u32,
                                             point:       u32,
                                             nsubdiv:     u32,
                                             out:         &mut Vec<Pnt3<u32>>) {
    assert!(nsubdiv > 0);

    for i in 0 .. nsubdiv - 1 {
        out.push(Pnt3::new(base_circle + i, point, base_circle + i + 1));
    }
}

/// Pushes indices so that a circle is filled with triangles. Each triangle will have the
/// `base_circle` point in common.
/// Pushes `nsubdiv - 2` elements to `out`.
#[inline]
pub fn push_filled_circle_indices(base_circle: u32, nsubdiv: u32, out: &mut Vec<Pnt3<u32>>) {
    for i in base_circle + 1 .. base_circle + nsubdiv - 1 {
        out.push(Pnt3::new(base_circle, i, i + 1));
    }
}

/// Given four corner points, pushes to two counterclockwise triangles to `out`.
///
/// # Arguments:
/// * `ul` - the up-left point.
/// * `dl` - the down-left point.
/// * `dr` - the down-left point.
/// * `ur` - the up-left point.
#[inline]
pub fn push_rectangle_indices<T: Clone>(ul: T, ur: T, dl: T, dr: T, out: &mut Vec<Pnt3<T>>) {
    out.push(Pnt3::new(ul.clone(), dl, dr.clone()));
    out.push(Pnt3::new(dr        , ur, ul));
}

/// Reverses the clockwising of a set of faces.
#[inline]
pub fn reverse_clockwising(indices: &mut [Pnt3<u32>]) {
    for i in indices.iter_mut() {
        mem::swap(&mut i.x, &mut i.y);
    }
}

/// Duplicates the indices of each triangle on the given index buffer.
///
/// For example: [ (0.0, 1.0, 2.0) ] becomes: [ (0.0, 0.0, 0.0), (1.0, 1.0, 1.0), (2.0, 2.0, 2.0)].
#[inline]
pub fn split_index_buffer(indices: &[Pnt3<u32>]) -> Vec<Pnt3<Pnt3<u32>>> {
    let mut resi = Vec::new();

    for vertex in indices.iter() {
        resi.push(
            Pnt3::new(
                Pnt3::new(vertex.x, vertex.x, vertex.x),
                Pnt3::new(vertex.y, vertex.y, vertex.y),
                Pnt3::new(vertex.z, vertex.z, vertex.z)
                )
            );
    }

    resi
}

/// Duplicates the indices of each triangle on the given index buffer, giving the same id to each
/// identical vertex.
#[inline]
pub fn split_index_buffer_and_recover_topology<P: PartialEq + AsBytes + Clone>(
                                               indices: &[Pnt3<u32>],
                                               coords:  &[P])
                                               -> (Vec<Pnt3<Pnt3<u32>>>, Vec<P>) {
    let mut vtx_to_id  = HashMap::new();
    let mut new_coords = Vec::with_capacity(coords.len());
    let mut out        = Vec::with_capacity(indices.len());

    fn resolve_coord_id<P: PartialEq + AsBytes + Clone>(
                        coord:      &P,
                        vtx_to_id:  &mut HashMap<HashablePartialEq<P>, u32>,
                        new_coords: &mut Vec<P>)
                        -> u32 {
        let key = unsafe { HashablePartialEq::new(coord.clone()) };
        let id = match vtx_to_id.entry(key) {
            Entry::Occupied(entry) => entry.into_mut(),
            Entry::Vacant(entry)   => entry.insert(new_coords.len() as u32)
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

        out.push(
            Pnt3::new(
                Pnt3::new(va, oa, oa),
                Pnt3::new(vb, ob, ob),
                Pnt3::new(vc, oc, oc)
                )
            );
    }

    new_coords.shrink_to_fit();

    (out, new_coords)
}

/// Computes the normals of a set of vertices.
#[inline]
pub fn compute_normals<P>(coordinates: &[P], faces: &[Pnt3<u32>], normals: &mut Vec<P::Vect>)
    where P: Point,
          P::Vect: Vect + Cross<CrossProductType = <P as Point>::Vect> {
    let mut divisor: Vec<<P::Vect as Vect>::Scalar> = iter::repeat(na::zero()).take(coordinates.len()).collect();

    // Shrink the output buffer if it is too big.
    if normals.len() > coordinates.len() {
        normals.truncate(coordinates.len())
    }

    // Reinit all normals to zero.
    normals.clear();
    normals.extend(iter::repeat(na::zero::<P::Vect>()).take(coordinates.len()));

    // Accumulate normals ...
    for f in faces.iter() {
        let edge1  = coordinates[f.y as usize] - coordinates[f.x as usize];
        let edge2  = coordinates[f.z as usize] - coordinates[f.x as usize];
        let cross  = na::cross(&edge1, &edge2);
        let normal;

        if !na::is_zero(&cross) {
            normal = na::normalize(&cross)
        }
        else {
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
